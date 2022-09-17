#!/usr/bin/env python

# ROS dependencies
from cProfile import label
from tkinter import Frame
from cv2 import circle
import rospy
from std_msgs.msg import Int32MultiArray

# Public dependencies
import matplotlib.pyplot as plt
from  matplotlib import patches
import numpy as np
np.set_printoptions(threshold=np.inf)
import os

class APF_Avoider():
    def __init__(self, ros_enabled, local_scope=None, AF_scaler=None, RF_scaler=None, RF_threshold=None):
        # public 
        self.pos = None # for testing
        self.goal = None
        self.gridmap = None
        self.map_height = None
        self.map_width = None
        self.local_scope = local_scope # 2D tuple
        self.local_map_center = None
        self.AF_scaler = AF_scaler
        self.RF_scaler = RF_scaler
        self.RF_threshold = RF_threshold
        self.APF = None
        self.motion = 8*np.array([[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]])

        self.pos = [10,100] # for test
        self.local_scope = 8*np.array([10,15]) # for test (even num required)
        self.AF_scaler = 80
        self.RF_scaler = 400
        self.RF_threshold = 5

        # ROS init
        if ros_enabled:
            rospy.init_node("APF_Avoider", anonymous=False)
            self.pub = rospy.Publisher("RefinedWaypoints", Int32MultiArray, queue_size=100)
            self.sub_gridmap = rospy.Subscriber("FakeMap", Int32MultiArray, callback=self.callback_get_map)
            self.sub_waypoints = rospy.Subscriber("FakeWaypoints", Int32MultiArray, callback=self.callback_get_waypoints)
            self.sub_robot_pos = rospy.Subscriber("RefinedWaypoints", Int32MultiArray, callback=self.callback_get_robot_pos)

    # TODO: In the future, this function should be splited as
    # 1. Subscribe the map ONCE
    # 2. Keep subcribing the obs_loc in real-time
    def callback_get_map(self, msg_received):
        # ObsDetection operates in high-freq

        # get robot_loc
        if self.pos == None:
            rospy.loginfo("Waiting for RobotLocation from Perception.")
            return
        # get waypoints
        if self.goal == None:
            rospy.loginfo("Waiting for Waypoints from Global Planning.")
            return
        
        gridmap = msg_received.data  # get map
        self.map_height = msg_received.layout.dim[0].size
        self.map_width = msg_received.layout.dim[1].size
        self.gridmap = np.array(gridmap).reshape(self.map_height,self.map_width)

        ax = self.generate_APF() # generate APF
        refined_waypoints = self.APF_planning() # execute obstacle avoidance
        goal = patches.Rectangle([refined_waypoints[1], refined_waypoints[0]], 2, 2, linewidth=2, edgecolor='c',facecolor='none', label="Goal") # Create a Rectangle patch
        ax.add_patch(goal) # Add the patch to the Axes
        plt.pause(0.01)
        self.publish(refined_waypoints)

    def callback_get_waypoints(self, msg_received):
        # GlobalPlanning operates in low-freq
        self.goal = msg_received.data
    
    def callback_get_robot_pos(self, msg_received):
        refined_waypoints = msg_received.data # 2D Tuple
        self.pos = refined_waypoints
        rospy.loginfo("Reached Waypoint = %s", self.pos)
    
    def run(self):
        rospy.loginfo("Obstacle_Avoidance starts.")
        rospy.spin()
    
    def publish(self, refined_waypoints):
        msg_to_send = Int32MultiArray()
        msg_to_send.data = refined_waypoints.reshape(-1)

        self.pub.publish(msg_to_send)

    def generate_APF(self):
        # init local map
        row_min = np.max([self.pos[0] - self.local_scope[0]/2, 0])
        row_max = np.min([self.pos[0] + self.local_scope[0]/2, self.map_height-1]) 
        col_min = np.max([self.pos[1] - self.local_scope[1]/2, 0])
        col_max = np.min([self.pos[1] + self.local_scope[1]/2, self.map_width-1])
        current_local_scope = [row_max - row_min, col_max - col_min]
        local_map = self.gridmap[row_min:row_max, col_min:col_max]
        # rospy.loginfo("%s, %s, %s, %s", row_min, row_max, col_min, col_max)
        rospy.loginfo("Locaol Map Size = %s", local_map.shape)
        self.local_map_center = np.array([self.pos[0]-row_min, self.pos[1]-col_min])
        rospy.loginfo("Locaol Map Center = %s", self.local_map_center)

        XX, YY = np.meshgrid(np.arange(current_local_scope[0]), np.arange(current_local_scope[1]), indexing = 'ij')
        XX -= self.local_map_center[0]
        YY -= self.local_map_center[1]

        # for all obs/boundary in local map
        RF = np.zeros_like(XX)
        num_obs = np.sum(local_map)
        rospy.loginfo("num_obs = %s", num_obs)
        for idx_row in range(current_local_scope[0]):
            for idx_col in range(current_local_scope[1]):
                if local_map[idx_row, idx_col] == 1:
                    obs = [idx_row, idx_col]
                    # (-)RepulsiveField generation (origin sits at RobotPosition)
                    relative_obs = obs - self.local_map_center
                    dist_to_obs = (XX-relative_obs[0])**2 + (YY-relative_obs[1])**2 + 0.1
                    # rospy.loginfo("RF for one obstacle generated!")
                    RF = RF + self.RF_scaler * (1/dist_to_obs - 1/self.RF_threshold)
        RF[RF < 0] = 0

        # (+)AttractiveField generation (origin sits at RobotPosition)
        relative_goal = np.array(self.goal) - np.array(self.pos)
        AF = self.AF_scaler * np.sqrt((XX-relative_goal[0])**2 + (YY-relative_goal[1])**2)
        self.APF = AF+RF
        rospy.loginfo("APF for Local Map Generated :)")
        rospy.loginfo("APF for Local Map Size = %s", self.APF.shape)

        global frame_num
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
        ax = plt.gca() # Get the current reference
        ax.clear()
        rect = patches.Rectangle([col_min, row_min], current_local_scope[1], current_local_scope[0],linewidth=2, edgecolor='r', facecolor='none') # Create a Rectangle patch
        center = patches.Rectangle([self.pos[1], self.pos[0]], 2, 2, linewidth=2, edgecolor='b',facecolor='none', label="Robot") # Create a Rectangle patch
        final = patches.Rectangle([self.goal[1], self.goal[0]], 5, 5, linewidth=6, edgecolor='g',facecolor='none', label="Goal") # Create a Rectangle patch
        ax.add_patch(rect) # Add the patch to the Axes
        ax.add_patch(center) # Add the patch to the Axes
        ax.add_patch(final) # Add the patch to the Axes
        plt.imshow(self.gridmap, cmap='Greys')
        plt.title("Global Scope")
        plt.tight_layout()

        # script_dir = os.path.dirname(__file__) # <-- absolute dir the script is in
        # rel_path = "apf_dynamics/"+ str(frame_num)
        # abs_file_path = os.path.join(script_dir, rel_path)
        # plt.savefig(abs_file_path)
        # frame_num += 1
        plt.pause(0.01)

        return  ax
        
    
    # Remaining Problems:
    # 1. No Distinguishment for the Wall(Boundary) and the Obstacle. Robot is likely to overreact to the walls.
    # 2. Only One-Grid-Movement in one Itertion
    def APF_planning(self):
        lowest_potential = self.APF[self.local_map_center[0],self.local_map_center[1]]
        future_move = np.array([0,0])

        for single_move in self.motion:
            new_pos = self.local_map_center + single_move
            if self.APF[new_pos[0],new_pos[1]] < lowest_potential:
                lowest_potential = self.APF[new_pos[0],new_pos[1]]
                future_move = single_move
        rospy.loginfo("Marching towards the Orientation %s", future_move)

        return future_move + self.pos

if __name__ == '__main__':

    frame_num = 0

    ros_enabled = True
    apf_avoider = APF_Avoider(ros_enabled)
    apf_avoider.run()