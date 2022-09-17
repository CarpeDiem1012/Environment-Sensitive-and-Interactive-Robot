#!/usr/bin/env python

# Public dependencies
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import matplotlib.pyplot as plt
from  matplotlib import patches
import numpy as np
np.set_printoptions(threshold=np.inf)

# ROS dependencies
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image


class APF_Avoider():
    ''' A cupsulated class for APF planning.
    @ Tips: For debuging, please match all key words using "Uncomment to debug" and uncomment them.
    '''
    def __init__(self, ros_enabled, local_scope_scaler=None, AF_scaler=None, RF_scaler=None, RF_threshold=None):
        # public 
        self.pos = None # robot_pos (pix) @ map_topleft
        self.waypoints = None # 1-D array (pix) @ map_center
        self.goal = None # top tuple (pix) @ map_center
        self.goalIterator = None
        self.gridmap = None
        self.map_height = None
        self.map_width = None
        self.map_center = None
        self.map_scaler = None
        self.local_scope_scaler = local_scope_scaler # 2D tuple
        self.local_scope = None
        self.local_map_center = None
        self.AF_scaler = AF_scaler
        self.RF_scaler = RF_scaler
        self.RF_threshold = RF_threshold
        self.APF = None
        self.motion = 20*np.array([[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]])

        '''for testing'''
        self.local_scope_scaler = np.array([0.2,0.2])
        self.AF_scaler = 100
        self.RF_scaler = 300
        self.RF_threshold = 5
        '''for testing'''

        # ROS init
        if ros_enabled:
            rospy.init_node("obstacle_avoidance", anonymous=False)
            self.ref_waypoints_pub = rospy.Publisher("refined_waypoints", Float32MultiArray, queue_size=100)
            self.HRI_error_pub = rospy.Publisher("human_robot_interaction_node/error_message", String, queue_size=1)

            self.gridmap_sub = rospy.Subscriber("image_process/occupancy_map", Image, callback=self.callback_get_map)
            self.waypoints_sub = rospy.Subscriber("global_waypoints", Int32MultiArray, callback=self.callback_get_waypoints)
            self.robot_pos_sub = rospy.Subscriber("image_process/husky_location", Point, callback=self.callback_get_robot_pos)
            self.FSM_sub = rospy.Subscriber("human_robot_interaction_node/FSM", String, queue_size=1)

    # TODO: In the future, this function should be splited as
    # 1. Subscribe the map ONCE
    # 2. Keep subcribing the obs_loc in real-time
    def callback_get_map(self, msg_received):
        ''' ObstacleDetection Operates in High-Freq'''
        # get robot_loc
        if self.pos is None:
            rospy.loginfo("[Obstacle Avoidance] Waiting for RobotLocation from Perception.")
            return

        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg_received, "bgr8")
        except CvBridgeError as cverror:
            print(cverror)

        self.gridmap = np.array(cv_image[:,:,0]).astype(int)
        self.map_height =  self.gridmap.shape[0]
        self.map_width = self.gridmap.shape[1]
        self.map_center = np.array([self.map_height//2, self.map_width//2])
        self.local_scope = self.local_scope_scaler * np.array([self.map_height, self.map_width])
        self.local_scope = np.round(self.local_scope)
        rospy.loginfo("[Obstacle Avoidance] Succesfully Receive the Map!")

        # get waypoints
        if self.goal is None:
            rospy.loginfo("[Obstacle Avoidance] Waiting for Waypoints from Global Planning.")
            return
        
        ax = self.generate_APF() # generate APF
        refined_waypoints = self.APF_planning() # execute obstacle avoidance
        goal = patches.Rectangle([refined_waypoints[1], refined_waypoints[0]], 2, 2, linewidth=2, edgecolor='g',facecolor='none', label="Goal") # Create a Rectangle patch
        ax.add_patch(goal) # Add the patch to the Axes
        self.publish_ref_waypoints(refined_waypoints)


    def callback_get_waypoints(self, msg_received):
        ''' Get GlobalWaypoints in low-freq, get the globalWaypoints list
        Args:
            msg_received (Float{"data": }): 1-D list with [goal_x, goal_y] (pix), x=col, y=row, origin=map_center
        '''
        # TODO: add an odd number check before reshaping
        self.waypoints = -np.array(msg_received.data).reshape(-1,2) + self.map_center
        rospy.loginfo("[Obstacle Avoidance] waypoints.shape = %s", self.waypoints.shape) # Uncomment to debug
        rospy.loginfo("[Obstacle Avoidance] map_center = %s", self.map_center) # Uncomment to debug

        self.waypoints = self.waypoints.astype(int)
        self.waypoints[self.waypoints<0] = 0
        # rospy.loginfo("[Obstacle Avoidance] waypoints.shape = %s", self.waypoints.shape) # Uncomment to debug
        # rospy.loginfo("[Obstacle Avoidance] waypoints.type = %s", type(self.waypoints)) # Uncomment to debug
        self.goalIterator = iter(self.waypoints)
        self.goal = next(self.goalIterator)


    def callback_get_robot_pos(self, msg_received):
        '''Get RobotPos in high-freq, check arrival and pop the top in globalWaypoints list
        Args:
            msg_received (Point{"x": "y": "z":}): (0,0)=topleft, x=col, y=row ,pixel/meter=z
        '''
        self.map_scaler = np.array(msg_received.z) # pixel/meter
        pos_in_meter = np.array([msg_received.y, msg_received.x]) # (0,0)=topleft, col=x, row=y
        self.pos = np.round(pos_in_meter * self.map_scaler).astype(int) # 2D Tuple
        rospy.loginfo("[Obstacle Avoidance] Current Pos = %s [pix]", self.pos)
        rospy.loginfo("[Obstacle Avoidance] Current Goal = %s [pix]", self.goal)
        rospy.loginfo_once("[Obstacle Avoidance] Current Map Scaler = %s [pixels/m]", self.map_scaler)

        # only acitvated when self.goal is SET
        if self.goal is not None:
            if np.linalg.norm((self.goal - self.pos), ord=2)/self.map_scaler < 0.1:
                    try: 
                        self.goal = next(self.goalIterator) # switch to next waypoint
                        rospy.loginfo("[Obstacle Avoidance] Current Goal Achieved. Switch to the Next Waypoint.")
                    except StopIteration:
                        rospy.loginfo("[Obstacle Avoidance] Iterated Over All Waypoints. Standing by wtih Alert.")
        else:
            rospy.loginfo("[Obstacle Avoidance] Waiting for Waypoints from Global Planning.")


    def publish_ref_waypoints(self, refined_waypoints):
        ''' publish absolute pos relative to the Gazebo global frame @ map_center
        '''
        refined_waypoints = -(refined_waypoints - self.map_center).astype(float) / self.map_scaler
        rospy.loginfo("Now Heading to Refined Waypoint %s (m) @ GazeboFrame", refined_waypoints)
        # TODO: check the indx out of bounds
        msg_to_send = Float32MultiArray()
        msg_to_send.data = refined_waypoints.reshape(-1)
        self.ref_waypoints_pub.publish(msg_to_send)


    def generate_APF(self):
        # init local map
        row_max = np.min([self.pos[0] + np.round(self.local_scope[0]/2), self.map_height-1]).astype(int)
        row_min = np.max([self.pos[0] - np.round(self.local_scope[0]/2), 0]).astype(int)
        col_min = np.max([self.pos[1] - np.round(self.local_scope[1]/2), 0]).astype(int)
        col_max = np.min([self.pos[1] + np.round(self.local_scope[1]/2), self.map_width-1]).astype(int)
        # rospy.loginfo("[Obstacle Avoidance] %s, %s, %s, %s", row_min, row_max, col_min, col_max) # Uncomment to debug
        current_local_scope = [row_max - row_min, col_max - col_min]
        local_map = self.gridmap[row_min:row_max, col_min:col_max]
        # rospy.loginfo("[Obstacle Avoidance] Locaol Map Size = %s", local_map.shape)
        self.local_map_center = np.array([self.pos[0]-row_min, self.pos[1]-col_min])
        # rospy.loginfo("[Obstacle Avoidance] Locaol Map Center = %s [pix]", self.local_map_center) # Uncomment to debug

        XX, YY = np.meshgrid(np.arange(current_local_scope[0]), np.arange(current_local_scope[1]), indexing = 'ij')
        XX -= self.local_map_center[0]
        YY -= self.local_map_center[1]

        # for all obs/boundary in local map
        RF = np.zeros_like(XX)
        num_obs = np.count_nonzero(local_map == 255)
        # rospy.loginfo("[Obstacle Avoidance] num_obs = %s", num_obs) # Uncomment to debug
        for idx_row in range(current_local_scope[0]):
            for idx_col in range(current_local_scope[1]):
                if local_map[idx_row, idx_col] == 255:
                    obs = [idx_row, idx_col]
                    # (-)RepulsiveField generation (origin sits at RobotPosition)
                    relative_obs = obs - self.local_map_center
                    dist_to_obs = (XX-relative_obs[0])**2 + (YY-relative_obs[1])**2 + 0.1
                    # rospy.loginfo("[Obstacle Avoidance] RF for one obstacle generated!")
                    RF = RF + self.RF_scaler * (1/dist_to_obs - 1/self.RF_threshold)
        RF[RF < 0] = 0

        # (+)AttractiveField generation (origin sits at RobotPosition)
        relative_goal = np.array(self.goal) - np.array(self.pos)
        AF = self.AF_scaler * np.sqrt((XX-relative_goal[0])**2 + (YY-relative_goal[1])**2)
        self.APF = AF+RF
        # rospy.loginfo("[Obstacle Avoidance] APF for Local Map Generated :)") # Uncomment to debug
        # rospy.loginfo("[Obstacle Avoidance] APF for Local Map Size = %s", self.APF.shape) # Uncomment to debug

        plt.figure("APF Monitor")
        plt.subplot(2,4,1)
        plt.imshow(AF, vmax=np.max(AF), cmap=plt.cm.Blues)
        plt.title("Attractive Field")
        plt.tight_layout()

        plt.subplot(2,4,2)
        plt.imshow(RF, vmax=np.max(RF), cmap=plt.cm.Blues)
        plt.title("Repulsive Field")
        plt.tight_layout()

        plt.subplot(2,4,5)
        plt.imshow(self.APF, vmax=np.max(self.APF), cmap=plt.cm.Blues)
        plt.title("APF")
        plt.tight_layout()

        plt.subplot(2,4,6)
        plt.imshow(local_map, cmap='Greys')
        plt.title("Local Scope")
        plt.tight_layout()

        plt.subplot(1,2,2)
        ax = plt.gca() # Get the current axis
        ax.clear()
        rect = patches.Rectangle([col_min, row_min],current_local_scope[1],current_local_scope[0],linewidth=2,edgecolor='r',facecolor='none') # Create a Rectangle patch
        center = patches.Rectangle([self.pos[1], self.pos[0]], 2, 2, linewidth=2, edgecolor='b',facecolor='none', label="Robot") # Create a Rectangle patch
        ax.add_patch(rect) # Add the patch to the Axes
        ax.add_patch(center) # Add the patch to the Axes
        plt.imshow(self.gridmap, cmap='Greys')
        plt.title("Global Scope")
        plt.tight_layout()
        plt.pause(0.01)

        return  ax
    

    # TODO:
    # 1. No Distinguishment for the Wall(Boundary) and the Obstacle. Robot is likely to overreact to the walls.
    # 2. Only One-Grid-Movement in one Itertion
    def APF_planning(self):
        lowest_potential = self.APF[self.local_map_center[0] ,self.local_map_center[1]]
        future_move = np.array([0,0])

        for single_move in self.motion:
            new_pos = self.local_map_center + single_move
            if self.APF[new_pos[0],new_pos[1]] < lowest_potential:
                lowest_potential = self.APF[new_pos[0],new_pos[1]]
                future_move = single_move
        rospy.loginfo("[Obstacle Avoidance] Marching towards the Orientation %s", future_move)

        return future_move + self.pos
    

    def run(self):
        rospy.loginfo("[Obstacle Avoidance] Obstacle_Avoidance Starts.")
        rospy.spin()


if __name__ == '__main__':

    ros_enabled = True
    apf_avoider = APF_Avoider(ros_enabled)
    apf_avoider.run()