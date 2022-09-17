#!/usr/bin/env python3 

## Ros dependencies
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageMsg

## Other dependencies
import numpy as np

## Google optimization tools
import six
import sys
import os
sys.modules['sklearn.externals.six'] = six
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

sys.setrecursionlimit(10000)

class PathPlanning():
    def __init__(self, ros_enabled, husky_size, start_position_husky, geometric_distance, waypoint_scale=2, solver='custom'):
        #Launch Client and wait for server
        
        self.ros_topic = 'global_waypoints'
        self.ros_enabled = ros_enabled

        self.geometric_distance = geometric_distance
        self.waypoint_scale = waypoint_scale
        self.solver = solver

        if ros_enabled:
            rospy.init_node('path_planning', anonymous=True)
            self.pub = rospy.Publisher(self.ros_topic, Int32MultiArray, queue_size=10)
            self.r = rospy.Rate(0.2) 
            self.r2 = rospy.Rate(0.2) 
            self.bridge = CvBridge()

        ## Husky parameters
        self.husky_size = husky_size
        self.start_position_husky = start_position_husky

        ### init lists or live params
        self.coverage = 0
        self.visited_waypoints = False

############### FINITE STATE MACHINE ##################
        
    def listener(self):
        rospy.Subscriber("/human_robot_interaction_node/FSM", String, self.callback_FSM)
        rospy.spin()


    def callback_FSM(self, data):
        if data.data == 'sweeping_state':
            rospy.Subscriber("/image_process/husky_location", Point, self.callback_localization)
            rospy.spin()
        elif data.data == 'low_battery_state':
            self.go_to_charging_hub()
        else:
            self.r.sleep()
        
############### Receiving position and scaling factor####################

    def callback_localization(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        x = int(np.round(data.x))
        y = int(np.round(data.y))

        self.length = data.z

        # If statements to fix the big deviations received from perception node
        
        if self.length > 10:
            self.husky_size = (int(np.round(self.length*0.7)), int(np.round(self.length)))
        else:
            self.length = 68
            self.husky_size = (int(68*0.7), 68)
        
        if (abs(x) > 10 or abs(y) > 10):
            self.start_position_husky = (430,420)
        else:
            self.start_position_husky = (int(x*self.length), int(self.length*y))
        self.start_husky_octomap = self.start_position_husky // np.min(self.husky_size)

        rospy.loginfo("The starting postion: "+str(self.start_position_husky))
        rospy.loginfo("Huskys size: "+str(self.husky_size))

        
        # Once finished listen to motion control waypoint to scrap them from planning
        # self.visited_waypoints = rospy.Subscriber("/motion_control/visited_waypoints", Int32MultiArray, self.callback_waypoints)

        rospy.Subscriber("/image_process/occupancy_map", ImageMsg, self.callback_image)
        rospy.spin()


############### Receive visted waypoints ###################

    def callback_waypoints(self, data):
        return data.data.reshape(2,-1)
        
############### START PLANNING SEQUENCE ####################

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "mono8":
            rospy.loginfo("ENCODING OF MESSAGE = "+str(img_msg.encoding))
            rospy.logerr("This Coral detect node has been hardcoded to the '"+str(img_msg.encoding)+"' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to mono 8 
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width), # and one channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        return image_opencv

    def callback_image(self, data):
        rospy.loginfo("Receiving image")
        try:
            self.cv_image = self.imgmsg_to_cv2(data)
            rospy.loginfo('Image Received in Path Planning node!')
        except CvBridgeError as e:
            print(e)
            pass

        rospy.loginfo('Start path planning')

        if self.solver == 'google':
            path = self.google_TSP(self.cv_image)
        else:
            path = self.hand_made_solver()
        
        

        self.publish(path)
        rospy.loginfo('Path planing complete, published to: '+ str(self.ros_topic))
        # self.animate`_path()

############# Publish path ###################

    def publish(self, input):
        input = np.array(input, dtype='int32')
        msg = Int32MultiArray()
        msg.data = input.reshape(-1)
        self.pub.publish(msg)


################### PLANNING HELP FUNCTION #####################

    def search_grid_generate(self):
        stride = int(self.husky_size[0]/self.waypoint_scale)
        self.stride = stride
        search_grid_image = self.cv_image.copy()
        y = int(stride//2)
        x = int(stride//2)
        search_grid_list_sub = []
        for i in np.arange(x, search_grid_image.shape[0]-x, stride):
            for j in np.arange(y, search_grid_image.shape[1]-y, stride):
                obstacles = 0
                for k in range(stride):
                    for l in range(stride):
                        if search_grid_image[int(i-int(stride//2)+k), int(j-int(stride//2)+l)] == 255:
                            obstacles += 1
                
                if obstacles < 6:
                    search_grid_list_sub.append(list([i,j]))
                    
        if self.visited_waypoints:
            #### Clean out the visited waypoints.
            search_grid_list_sub = search_grid_list_sub

        self.search_grid_list = np.array(search_grid_list_sub)

################### PLANNING HELP FUNCTION #####################

    def make_distance_matrix(self):
        self.distance_matrix = np.zeros((self.search_grid_list.shape[0], self.search_grid_list.shape[0]))
        rospy.loginfo('AMOUNT OF WAYPOINTS TO VISIT: '+str(self.distance_matrix.shape[0]))
        for i in range(self.search_grid_list.shape[0]):
            for j in range(self.search_grid_list.shape[0]):
                distance = np.sqrt(((self.search_grid_list[i,0]-self.search_grid_list[j,0])**2)*1+((self.search_grid_list[i,1]-self.search_grid_list[j,1])**2)*2)
                self.distance_matrix[i,j] = distance

####################################################

    def make_path_list(self, best_state):
        path_list = []
        for state in best_state:
            path_list.append(list(self.coord_list[state]))

        return np.array(path_list)

####################################################
   
    def make_geometrix_path_list(self, path_list):
        center = [int(self.cv_image.shape[0]//2), int(self.cv_image.shape[1]//2)]

        if self.length > 10:
            grid_size = self.length # Scaling factor from perception node
        else:
            grid_size = np.min(self.husky_size)*0.776 #Calibration facto 0.776 for when perception does not work
        
        geometric_path_list = []
        for waypoint in path_list:
            centered_position = -1*(waypoint - center)
            if self.geometric_distance:
                geo_position = centered_position/grid_size
            else:
                geo_position = centered_position
            geometric_path_list.append(list(geo_position))
 
        return np.array(geometric_path_list)

####################################################

    def depot_number(self):
        location = np.array([self.start_position_husky[0], self.start_position_husky[1]])

        grid_list = self.search_grid_list.copy()
        grid_list = np.asarray(grid_list)

        distance_to_start = np.sqrt(np.sum((grid_list - location)**2, axis=1))
        idx = distance_to_start.argmin()
        return grid_list[idx], idx

#######################################################

    def create_line_plot_path(self,path_list):
        cv_image = cv2.cvtColor(self.cv_image.copy(),cv2.COLOR_GRAY2RGB)
        scale = np.min(self.husky_size)
        x1 = int(path_list[0,0])
        y1 = int(path_list[0,1])
        cv2.circle(cv_image, (y1,x1), radius=7, color=(255,255,0), thickness=-1)

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (y1+5, x1+2)
        fontScale              = 0.4
        fontColor              = (255,0,255)
        thickness              = 1
        lineType               = 2

        cv2.putText(cv_image,'0', 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)
        
        for i in range(path_list.shape[0]-1):
            x2 = int(path_list[i+1,0])
            y2 = int(path_list[i+1,1])
            cv2.line(cv_image, (y1, x1), (y2, x2), (255,255,0), thickness=2)
            cv2.circle(cv_image, (y2, x2), radius=5, color=(0,255,255), thickness=-1)
            cv2.putText(cv_image, str(i+1), 
                (y2+5, x2+2), 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)
            x1 = x2
            y1 = y2

        cv2.imshow('path_planning_route', cv_image)
        cv2.waitKey(5000)
        cv2.imwrite(os.path.dirname(__file__)+'/path_list.jpg', cv_image)

#######################################################

    def create_animation_images(self, path_list):
        animation_slides = self.occupancy_grid
        ## Intial position as first slide
        animation_slides[self.start_husky_octomap[0],self.start_husky_octomap[1]] = 90
        self.animation_video.append(np.array(animation_slides))
        animation_slides[self.start_husky_octomap[0],self.start_husky_octomap[1]] = 175

        ## Make a slide deck of all positions in order of path list
        for i in range(path_list.shape[0]):
            animation_slides[path_list[i,0],path_list[i,1]] = 90
            self.animation_video.append(np.array(animation_slides))
            animation_slides[path_list[i,0],path_list[i,1]] = 175

    def print_solution(self, manager, routing, solution):
        index_list = []

        print('Objective: {} miles'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route for vehicle 0:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            index_list.append(index-1)
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        plan_output += 'Route distance: {}miles\n'.format(route_distance)


        path_list = self.make_path_list(index_list)
        self.create_animation_images(path_list)
        self.create_line_plot_path(path_list)

        return path_list

######################################################
############# PLANNING MAIN FUNCTION #################
######################################################

    def google_TSP(self, map):
        ## Waypopints for the solver
        self.search_grid_generate()

        """Entry point of the program."""
        ## https://developers.google.com/optimization/routing/tsp 
        # Instantiate the data problem.
        self.make_distance_matrix()
        _, depot_number = self.depot_number()

 
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(self.distance_matrix.shape[0], 1, int(depot_number))

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)


        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)

        ## Some settings for optimization
        # routing.AddDimension(transit_callback_index,10,10,True,'dim1')

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)
        

        # Print solution on console.
        if solution:
            rospy.loginfo('Path Solution Found!!')
            path = self.print_solution(manager, routing, solution)        

        path = self.make_geometrix_path_list(path)

        return path

##########################################################
############# Handmade solutions #################
##########################################################

    def hand_made_solver(self):
        self.search_grid_generate()
        self.make_distance_matrix()

        distance_matrix_edit = self.distance_matrix.copy()
        grid_list = self.search_grid_list.copy()

        depot_location, depot_number = self.depot_number()
        prev_location = depot_number
        path_list = []
        index_list = []
        path_list.append(list(depot_location))
        grid_list = np.delete(grid_list, prev_location, axis=0)
        distance_matrix_edit = np.delete(distance_matrix_edit, prev_location, axis=1)
        
        for i in range(self.distance_matrix.shape[0]-1):
            # Find closest location to current location and append to the list
            next_location = np.argmin(distance_matrix_edit[prev_location,:])
            path_list.append(list(grid_list[next_location]))

            ## Delete the prev from row and next from column and also delete the next from the waypoint list
            grid_list = np.delete(grid_list, next_location, axis=0)
            distance_matrix_edit = np.delete(distance_matrix_edit, next_location, axis=1)
            distance_matrix_edit = np.delete(distance_matrix_edit, prev_location, axis=0)

            prev_location = next_location


        path_list.append(list(depot_location))
        path_list = np.array(path_list)
        # path_list = self.search_grid_list

        self.create_line_plot_path(path_list)
        path = self.make_geometrix_path_list(path_list)
        rospy.loginfo("The first three Waypoints: \n"+str(path[:3]))

        return path


##########################################################
############# Return to charging station #################
##########################################################

    def go_to_charging_hub(self):
        base_location = np.array(self.make_geometrix_path_list(self.start_position_husky), dtype='int32').reshape(-1)
        msg = Int32MultiArray()
        msg.data = base_location
        self.pub.publish(msg)


###############
### __MAIN__ ##
###############

if __name__ == "__main__":
    husky_size = (21,30)
    start_position_husky = (430,420)
    ros_enabled = True
    geometric_distance = True


    #Choose your solver, custom recommended
    # solver = 'google'
    solver = 'custom'


    path_planning = PathPlanning(
        ros_enabled = ros_enabled,
        husky_size = husky_size,
        start_position_husky = start_position_husky,
        geometric_distance = geometric_distance,
        waypoint_scale = 2,
        solver = solver
    )

    path_planning.listener()