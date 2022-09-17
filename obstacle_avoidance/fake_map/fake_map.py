#!/usr/bin/env python

# ROS dependencies
from scipy import rand
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

# Other dependencies
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(threshold=np.inf)
from PIL import Image
import random
import os

class FakeMap():
    def __init__(self, ros_enabled):
        # public
        self.map = None # GrayScale Map after Cutting
        self.grid = None # Binary Map after Filtering
        self.obs_num = None
        self.obs_list = None # List of Obs, Obs.dtype = dic
        self.width = None
        self.height = None

        # Launch Node
        if ros_enabled:
            rospy.init_node('FakeMapGenerator', anonymous=False)
            self.pub = rospy.Publisher('FakeMap', Int32MultiArray, queue_size=100)
    
    def edg_cutting(self, img):
        topleft = np.array([img.shape[0], img.shape[1]])
        botright = np.array([0,0])

        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i,j] != 255:
                    if i <= topleft[0] and j <= topleft[1]:
                        topleft = np.array([i,j])
                    if i >= botright[0] and j >= botright[1]:
                        botright = np.array([i,j])

        [self.height, self.width] = botright - topleft
        self.map = img[topleft[0]:botright[0], topleft[1]:botright[1]]

    def boundary_representation(self):
        self.grid = np.ones_like(self.map)
        self.grid[self.map == 255] = 0 
        self.grid[self.map != 255] = 1

    def add_dynamic_obstacles(self, num):
        self.obs_num = num
        self.obs_list = []
        while(num):
            # random center
            [self.grid==0]
            # obs =   {'center': [random.random()*self.height, random.random()*self.width],
            #         'radius': 10
            #         }
            obs =   {'center': [85, 215 + (random.random()-0.5)*60], 
                    'radius': 10
                    }
            self.obs_list.append(obs)
            num -= 1
    
    def move_dynamic_obstacles(self):
        # change the self.obs
        num = len(self.obs_list)
        while(num):
            obs = self.obs_list[num-1]
            obs['center'][0] = obs['center'][0] + (random.random()-0.5)*20
            obs['center'][1] = obs['center'][1] + (random.random()-0.5)*20
            # obs = self.obs_list[num-1]
            # new_center[0] = obs['center'][0] + (random.random()-0.5)*20
            # new_center[1] = obs['center'][1] + (random.random()-0.5)*20
            # if self.grid[new_center[0], new_center[1]]==0:
            #     obs['center'] = new_center 
            num -= 1
    
    def obstacle_representation(self):
        # copy and fill the obs, do not change self.obs
        gridmap = np.copy(self.grid)
        for i in range(self.height):
            for j in range(self.width):
                for obs in self.obs_list:
                    loc = obs['center']
                    radius = obs['radius']
                    if (i-loc[0])**2 + (j-loc[1])**2 <= radius**2:
                        gridmap[i,j]=1
        return gridmap

    def set_init(self, img, obs_num):
        self.edg_cutting(img)
        self.boundary_representation()
        self.add_dynamic_obstacles(obs_num)
        # return self.obstacle_representation()

    def get_renewed(self):
        self.move_dynamic_obstacles()
        return self.obstacle_representation()

    def publish(self):
        rospy.loginfo("Pulishing map from FakeMap")
        
        rate = rospy.Rate(1) # Global Planning Period = 10s
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            gridmap = self.obstacle_representation() # gridmap.dtype = np.ndarray
            self.move_dynamic_obstacles() # move the obs
            # plt.imshow(gridmap, cmap='Greys') # uncomment to vis map during ros
            # plt.pause(0.01) # uncomment to vis map during ros

            msg_to_pub = Int32MultiArray() # std_msg.MultiArray usage refers to http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html
            msg_to_pub.data = gridmap.reshape(-1) # invalid unless squeezed into 1D-array
            dim = []
            dim.append(MultiArrayDimension(label = "map_height", size = self.height))
            dim.append(MultiArrayDimension(label = "map_width", size = self.width))
            msg_to_pub.layout.dim = dim
            self.pub.publish(msg_to_pub)

            rospy.loginfo("Map Height = %s", self.height)
            rospy.loginfo("Map width = %s" , self.width)
            rospy.loginfo("Current Map Published Time = %s (s)", rospy.get_time() - start_time)
            rate.sleep()
            
# excuted only when being run as a script, not imported module
if __name__ == "__main__":

    script_dir = os.path.dirname(__file__) # <-- absolute dir the script is in
    rel_path = "lely_map.PNG"
    abs_file_path = os.path.join(script_dir, rel_path)
    img = Image.open(abs_file_path).convert('L')
    img = np.array(img)

    ros_enabled = True
    # ros_enabled = False

    fakemap = FakeMap(ros_enabled)
    fakemap.set_init(img, obs_num=3)

    if not ros_enabled:
        plt.imshow(fakemap.grid)
        # renew map in animation
        while(1):
            plt.clf() # clear the canvas
            gridmap = fakemap.get_renewed()
            # Ploting graph
            # [0,0] = topleft
            # [max,max] = botright
            # [x,y] = [row, locumn] 
            # gridmap[150:160,100:110]=1 # uncomment to use static_obs
            plt.imshow(gridmap, cmap='Greys')
            plt.pause(0.01)
        
    if ros_enabled:
        fakemap.publish()

