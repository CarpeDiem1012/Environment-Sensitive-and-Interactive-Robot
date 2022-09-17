#!/usr/bin/env python3 

## Ros dependencies
# from turtle import position
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Float32MultiArray, Int16MultiArray, MultiArrayDimension, Int16
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageMsg

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

## Other dependencies
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from PIL import Image
import random
from tqdm import tqdm
# from python_tsp.exact import solve_tsp_dynamic_programming, solve_tsp_brute_force
# from python_tsp.heuristics import solve_tsp_local_search, solve_tsp_simulated_annealing
import six
import sys
import os
sys.modules['sklearn.externals.six'] = six
import mlrose

sys.setrecursionlimit(10000)


## Google optimization tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp



class PathPlanning():
    def __init__(self, ros_enabled, husky_size, start_position_husky, map):
        #Launch Client and wait for server
        
        self.ros_topic = 'global_waypoints'
        self.ros_enabled = ros_enabled

        if ros_enabled:
            rospy.init_node('path_planning', anonymous=True)
            self.pub = rospy.Publisher(self.ros_topic,numpy_msg(Floats), queue_size=10)
            self.r = rospy.Rate(0.2) 
            self.r2 = rospy.Rate(0.2) 
            self.bridge = CvBridge()


            # self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            # rospy.loginfo("Path planning waiting for server")
            # self.client.wait_for_server()


        ## Husky parameters
        self.husky_size = husky_size
        self.start_position_husky = start_position_husky
        self.husky_size_cube = (np.min(husky_size), np.min(husky_size))

        ## occupancy map
        self.map = map
        # self.obstacle_representation(map)
        # self.occupancy_grid_generate(map)

        ### Params
        self.animation_video = []
        self.x_step = 0
        self.y_step = -2
        self.step_max_size = 3
        self.orientation_x = 0
        self.orientation_y = 0
        self.coverage = 0

        self.threshold = 0

        return None

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "MONO8":
            rospy.loginfo("ENCODING OF MESSAGE = "+str(img_msg.encoding))
            rospy.logerr("This Coral detect node has been hardcoded to the '32FC1' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 32 bit floats...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width), # and one channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        return image_opencv

    def callback(self, data):
        rospy.loginfo("Start Callback function")
        self.r.sleep()
        rospy.loginfo("Receiving image")
        try:
            # self.cv_image = self.bridge.imgmsg_to_cv2(data, '32FC1')
            self.cv_image = self.imgmsg_to_cv2(data)
            self.r2.sleep()
            rospy.loginfo('Image set in Path Planning node!')
        except CvBridgeError as e:
            print(e)
            pass

        self.occupancy_grid_generate()
        rospy.loginfo('Start path planning')
        path = self.google_TSP(self.cv_image)
        rospy.loginfo('Path calculated!')
        
        ####################
        ## Animation plot ##
        ####################
        self.animate_path(animate=True)

        if self.ros_enabled:
            rospy.loginfo('Ros enabled!')
            self.publish(path)
        
        print('Path planing complete')
        rospy.loginfo('Path planing complete, published to: ', self.ros_topic)
        print('Map fully covered!')
        rospy.loginfo('Map fully covered!')


        
    def listener(self):

        rospy.Subscriber("/image_process/occupancy_map", ImageMsg, self.callback)

        rospy.spin()
        

    def publish(self, input):
        rospy.loginfo('Starting publishing process.')

        input = np.array(input, dtype='float32')
        msg = Float32MultiArray()

        # rospy.loginfo(('Adding first dimension'))
        # msg.layout.dim.append(MultiArrayDimension())
        # msg.layout.dim[0].label = "Waypoints"
        # msg.layout.dim[0].size = input.shape[0]
        # msg.layout.dim[0].stride = 1

        # rospy.loginfo('Adding second dimension')
        # msg.layout.dim.append(MultiArrayDimension())``
        # msg.layout.dim[1].label = "x & y"
        # msg.layout.dim[1].size = input.shape[1]
        # msg.layout.dim[1].stride = 1

        msg.layout.data_offset = 0.0

        rospy.loginfo('Setting data in message')
        
        msg.data = []
        msg.data = list(input)

        self.pub.publish(input)
        print('Message published!')
        # rospy.spin()

    def run(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]
        goal.target_pose.pose.position.z = goal_pose[2]
        goal.target_pose.pose.orientation.x = goal_pose[3]
        goal.target_pose.pose.orientation.y = goal_pose[4]
        goal.target_pose.pose.orientation.z = goal_pose[5]
        goal.target_pose.pose.orientation.w = goal_pose[6]

        
        rospy.loginfo('Sending move base goal ..... ')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Navigation finished")

    def obstacle_representation(self,map):
        self.obstacles = np.ones_like(map)
        self.obstacles[map == 255] = 0 
        self.obstacles[map == 34] = 1

        # # Cut down the image up to the wall
        top_left = (np.min(np.where(self.obstacles==1)[0]), np.min(np.where(self.obstacles==1)[1]))
        top_right = (np.max(np.where(self.obstacles==1)[0]), np.max(np.where(self.obstacles==1)[1]))
        self.obstacles = self.obstacles[top_left[0]:top_right[0],top_left[1]:top_right[1]]
        self.obstacles[61:,85:185] = 1 #outside wall also as obstacles

####################################################

    def occupancy_grid_generate(self):
        
        self.occupancy_grid = self.cv_image.copy()
        plt.imshow(self.occupancy_grid)
        plt.show()

        self.occupancy_grid[self.occupancy_grid > np.min(self.occupancy_grid[np.nonzero(self.occupancy_grid)])] = 255
        plt.imshow(self.occupancy_grid)
        plt.show()

        self.occupancy_grid = cv2.resize(self.occupancy_grid, (self.cv_image.shape[1]//np.min(self.husky_size), self.cv_image.shape[0]//np.min(self.husky_size)), interpolation=cv2.INTER_NEAREST)
        self.occupancy_grid[13:,18:35] = 255 #outside wall also as obstacles
        plt.imshow(self.occupancy_grid)
        plt.show()

        self.start_husky_octomap = self.start_position_husky // np.min(self.husky_size)

        # cv2.imshow('Received image', self.occupancy_grid)
        # cv2.waitKey(5000)

####################################################

    def make_coords_list(self):
        nodes = np.where(self.occupancy_grid == 0)
        self.coord_list = np.array([nodes[0],nodes[1]]).T
        self.length_list = self.coord_list.shape[0]
        self.coord_list = self.coord_list[:self.length_list]
        print(self.start_husky_octomap)
        print(np.where(np.all(self.coord_list == self.start_husky_octomap, axis=1)))
        where_list = np.where(np.all(self.coord_list == self.start_husky_octomap, axis=1))
        print("Where_list: ", where_list)
        print("Where_list shape: ", where_list[0].shape)
        if where_list[0].shape[0] != 1:
            print('Depot Number Set to zero!')
            self.depot_number = 0
        else:
            print('Depot Number Set!')
            self.depot_number = where_list[0][0]

####################################################

    def make_distance_list(self):
        self.distance_list = []
        self.max_distance_nodes = 10
        for i in tqdm(range(self.coord_list.shape[0])):
            for j in range(self.coord_list.shape[0]):
                distance = np.sqrt((self.coord_list[i,0]-self.coord_list[j,0])**2+(self.coord_list[i,1]-self.coord_list[j,1])**2)
                
                if distance < self.max_distance_nodes and i != j:
                    self.distance_list.append([i,j,distance])
                elif i == 0 and j == self.coord_list.shape[0]-1:
                    self.distance_list.append([i,j,distance])
                elif j == 0 and i == self.coord_list.shape[0]-1:
                    self.distance_list.append([i,j,distance])

####################################################

    def make_distance_matrix(self):
        self.distance_matrix = np.zeros((self.coord_list.shape[0], self.coord_list.shape[0]))
        rospy.loginfo('AMOUNT OF WAYPOINTS TO VISIT: '+str(self.distance_matrix.shape[0]))
        for i in range(self.coord_list.shape[0]):
            for j in range(self.coord_list.shape[0]):
                distance = np.sqrt((self.coord_list[i,0]-self.coord_list[j,0])**2+(self.coord_list[i,1]-self.coord_list[j,1])**2)
                # if distance < 5:
                self.distance_matrix[i,j] = distance
                # else:
                    # self.distance_matrix[i,j] = 10**15

####################################################

    def make_path_list(self, best_state):
        path_list = []
        for state in best_state:
            path_list.append(list(self.coord_list[state]))

        return np.array(path_list)

####################################################
   
    def make_geometrix_path_list(self, path_list):
        center = [self.occupancy_grid.shape[0] // 2, self.occupancy_grid.shape[1] // 2]

        grid_size = 0.46 #700mm width of husky

        geometric_path_list = []
        for waypoint in path_list:
            centered_position = -1*(waypoint - center)
            geo_position = centered_position*grid_size
            geometric_path_list.append(list(geo_position))       
            
 
        return np.array(geometric_path_list)

####################################################

    def create_line_plot_path(self, path_list):
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(path_list[:,1], -1*path_list[:,0])
        # plt.show()

####################################################

    def create_line_plot(self, path_list):
        fig = plt.figure()
        ax = plt.axes()
        steps = np.linspace(1,path_list.shape[0], path_list.shape[0])

        ax.plot(steps, path_list)
        plt.show()

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

        print('animation slides ready!')



###############################
######### TSP mlrose ##########
###############################


    def TSP_plan_path(self,map):
        
        print(self.distance_list)

        ## Optimalization algo
        fitness_coords = mlrose.TravellingSales(distances=self.distance_list)
        problem_fit = mlrose.TSPOpt(length = self.coord_list.shape[0], fitness_fn = fitness_coords, maximize=False)

        # Random algo
        # problem_fit = mlrose.TSPOpt(length = self.length_list, coords = self.coord_list, maximize=False)   
        print(problem_fit)     
        best_state, best_fitness, curve = mlrose.mimic(problem_fit, max_iters=100, curve=True) # mutation_prob=0.2, max_attempts=20,

        print('best_state: ', best_state)
        print('best_fitness: ', best_fitness)

        self.create_line_plot(curve)

        path_list = self.make_path_list(best_state)
        self.create_animation_images(path_list)
        self.create_line_plot_path(path_list)
        # permutation, distance = solve_tsp_dynamic_programming(self.distance_matrix)
        # print('permutation: ', permutation)
        # print('distance: ', distance)

        return path_list

##########################################
############# Google TSP #################
##########################################

    def google_TSP(self, map):
        ## creat ooccupancy grid the size of husky
        self.make_coords_list()

        """Entry point of the program."""
        ## https://developers.google.com/optimization/routing/tsp 
        # Instantiate the data problem.
        self.make_distance_matrix()
 
        # Create the routing index manager.
        print("Entries distance matrix: ", self.distance_matrix.shape[0])
        print("Depot coordinates: ", (self.depot_number))
        manager = pywrapcp.RoutingIndexManager(self.distance_matrix.shape[0], 1, int(self.depot_number))

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
        print('Solution Found!!')

        # Print solution on console.
        if solution:
            path = self.print_solution(manager, routing, solution)        

        path = self.make_geometrix_path_list(path)

        return path
    
    def print_solution(self, manager, routing, solution):
        """Prints solution on console."""
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

#########################################################################
############## Random Algorithm own implementation ######################
#########################################################################


    def plan_path(self,map, husky_position, husky_size):
        self.obstacle_representation(map)
        pixels_covered = 0
        
        # Cut down the image up to the wall
        top_left = (np.min(np.where(self.obstacles==1)[0]), np.min(np.where(self.obstacles==1)[1]))
        top_right = (np.max(np.where(self.obstacles==1)[0]), np.max(np.where(self.obstacles==1)[1]))
        self.obstacles = self.obstacles[top_left[0]:top_right[0],top_left[1]:top_right[1]]
        self.obstacles[61:,85:185] = 1 #out wall also as obstacles]


        ## creat ooccupancy grid the size of husky
        self.occupancy_grid_generate()

        pixels_to_cover = self.obstacles.shape[0]*self.obstacles.shape[1] - np.count_nonzero(self.obstacles == 1)
        print('pixels to cover: ', pixels_to_cover)
        print('pixels that are obstacles', np.count_nonzero(self.obstacles == 1))
        print('obstacle map size', self.obstacles.shape[0]*self.obstacles.shape[1])

        # Params for measuring coverage
        map = self.obstacles
        print('total maps size: ',map.shape[0]*map.shape[1])
        self.coverage = pixels_covered/pixels_to_cover
        map_covered = False
        path = []

        plt.imshow(map)
        plt.show()

        self.i = 0
        self.collision_count  = 0
        while self.coverage < 0.95:
            # propose a new position
            husky_proposal_position = [husky_position[0] + self.x_step,husky_position[1] + self.y_step]

            # Check collision of new position
            husky_pixels = self.pixels_husky(self.obstacles, husky_proposal_position, husky_size)
            collision = self.check_collision(husky_pixels, self.obstacles)

            # update position if not in collision
            if collision == True:
                husky_position = husky_proposal_position
                path.append(np.array([husky_position[0],husky_position[1],self.i]))

                # Marking covered area with number 0.5
                map[husky_pixels == 1] = 2

                print('################################')
                print('counter steps:', self.i)
                print("obstacle count: ", np.count_nonzero(map == 1))
                print('coverage count: ', np.count_nonzero(map == 2))


                pixels_covered = np.count_nonzero(map == 2)
                print((pixels_covered),'/',pixels_to_cover)
                self.coverage = (pixels_covered)/pixels_to_cover
                print("New coverage: ", self.coverage)
            else:
                self.orientation_x += (random.randint(1,16)/8)*np.pi
                self.orientation_y += (random.randint(1,16)/8)*np.pi
                self.x_step = int(self.step_max_size*np.cos(self.orientation_x))
                self.y_step = int(self.step_max_size*np.sin(self.orientation_x))
                self.collision_count += 1 
                print("Collision!")
                pass

            self.i += 1

                
        print('Full coverage achieved!')
        return path
    
    def pixels_husky(self, obstacles, husky_position, husky_size):
        husky_pixels = np.zeros_like(obstacles)
        x_min = int(husky_position[0]-husky_size[0]//2)
        x_max = int(husky_position[0]+husky_size[0]//2+husky_size[0]%2)
        y_min = int(husky_position[1]-husky_size[1]//2)
        y_max = int(husky_position[1]+husky_size[1]//2+husky_size[1]%2)
        husky_pixels[y_min:y_max,x_min:x_max] = 1
        return husky_pixels



    def check_collision(self,husky_pixels, obstacles):

        obstacles = obstacles
        husky_in_map = np.array(obstacles)
        husky_in_map[husky_pixels == 1] = 0.2

        colliding_pixels = np.count_nonzero(obstacles[husky_pixels == 1] == 1)
        print("colliding pixels: ", colliding_pixels)

        if colliding_pixels <= 0:
            self.animation_video.append(np.array(husky_in_map))
            return True
        else:
            return False

########################################
########### Animation ##################
########################################

    def animate_path(self, animate):
        animation_deck = np.array([self.animation_video])[0]
        fps = 3
        nSeconds = 5
        # First set up the figure, the axis, and the plot element we want to animate
        fig = plt.figure()

        img = plt.imshow(animation_deck[0], cmap = "gray", vmin=0, vmax=255)

        def animate_func(i):
            if i % fps == 0:
                print( '.', end ='' )

            img.set_array(animation_deck[i])
            return [img]

        anim = animation.FuncAnimation(
                                    fig, 
                                    animate_func, 
                                    frames = animation_deck.shape[0],
                                    interval = 1000 / fps, # in ms
                                    )

        if animate == True:
            print('Saving animation...')
            anim.save(os.path.dirname(__file__) +'/test_anim.gif', fps=fps)
            print('Done!')
        elif animate == False:
            print('Animation not saved!')


###############
### __MAIN__ ##
###############

if __name__ == "__main__":
    
    # im = Image.open(os.getcwd()+'/src/cor_mdp_husky/path_planning/maps/lely_map.PNG')
    
    # im = Image.open('./src/cor_mdp_husky/path_planning/maps/lely_map.PNG')
    im = Image.open(os.path.dirname(__file__) + '/../maps/lely_map.PNG')
    # im = Image.open('/home/jesse/Documents/mdp/catkin_ws2/src/cor_mdp_husky/path_planning/maps/lely_map.PNG')
    print('Image Loaded!!')
    im = np.array(im)
    husky_size = (21,30) # 16 pixels length and 10 pixel width
    ## Starting pose: [0, 6.5, 0] this is in meters
    start_position_husky = (430,420)
    ros_enabled = True

    map = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    map = cv2.resize(map, (285, 174))

    path_planning = PathPlanning(
        ros_enabled = ros_enabled,
        husky_size = husky_size,
        start_position_husky = start_position_husky,
        map = map
    )

    path_planning.listener()