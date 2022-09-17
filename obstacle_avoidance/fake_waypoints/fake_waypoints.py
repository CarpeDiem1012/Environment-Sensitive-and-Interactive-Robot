#!/usr/bin/env python

## Ros dependencies
import rospy
from std_msgs.msg import Int32MultiArray

## Other dependencies
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(threshold=np.inf)

class FakeWaypoints():
    def __init__(self, ros_enabled):
        # Launch Node
        if ros_enabled:
            rospy.init_node('FakeWaypointsGenerator', anonymous=False)
            self.pub = rospy.Publisher('FakeWaypoints', Int32MultiArray, queue_size=100)
            self.sub = rospy.Subscriber('FakeMap', Int32MultiArray, self.callback_get_gridmap)

    def callback_get_gridmap(self, msg_received):
        map_height = msg_received.layout.dim[0].size
        map_width = msg_received.layout.dim[1].size
        gridmap = msg_received.data # receive as Tuple by default
        gridmap = np.array(gridmap).reshape(map_height, map_width)
        rospy.loginfo_once("Successfully Received Gridmap.")
        rospy.loginfo("Map Shape = %s", gridmap.shape)
        # plt.imshow(gridmap, cmap='Greys') # uncomment to vis map during ros
        # plt.pause(0.01) # uncomment to vis map during ros

        # processe the map, get waypoints
        waypoints = np.array([100,320]) # naive
        self.publish(waypoints)

    def publish(self,waypoints):
        rospy.loginfo_once("Publishing the Waypoints.")
        rospy.loginfo("Published Waypoint = %s", waypoints)
        msg_to_send = Int32MultiArray(data = waypoints)
        self.pub.publish(msg_to_send)

    def run(self):
        rospy.spin()

if __name__ == "__main__":

    ros_enabled = True

    fakewaypoints = FakeWaypoints(ros_enabled)
    fakewaypoints.run()
