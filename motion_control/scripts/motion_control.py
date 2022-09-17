#!/usr/bin/env python2

## Ros dependencies
import rospy
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import numpy as np

class Motion_Control:
		def __init__(self):
				# self.subwaypoints = rospy.Subscriber("global_waypoints", numpy_msg(Floats), self.goal)
				self.subodom = rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)
				self.pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size = 1)

				self.x = 0.0
				self.y = 0.0 
				self.theta = 0.0
				self.prev_waypoint = np.array([0.0,0.0])
				self.last_waypoint = False
				self.actual_angle = 0.0
				self.speed = Twist()
				self.ready = False
				self.r = rospy.Rate(10)

		def listener(self):
				self.substate = rospy.Subscriber("/human_robot_interaction_node/FSM", String, self.callbackFSM)
				
				rospy.spin()
		
		def newOdom(self, msg):
				self.x = msg.pose.pose.position.x
				self.y = msg.pose.pose.position.y + 6.5

				rot_q = msg.pose.pose.orientation
				(roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

		def run(self, goal_pose):

				while not rospy.is_shutdown() and (self.current_state == "sweeping_state" or self.current_state == "low_battery_state"):
					self.ready = False
					goal_x = goal_pose[0]
					goal_y = goal_pose[1]
					
					inc_x = goal_x - self.x
					inc_y = goal_y - self.y
					rospy.loginfo_throttle(1,'x: '+ str(self.x)+' and y: '+ str(self.y))
					angle_to_goal = atan2(inc_y, inc_x)
					
					#fix the angle if the angle of the robot and the directional angle don't allign
					if inc_x < 0:
						if angle_to_goal < 0 and self.theta > 0:
								actual_angle = np.pi - abs(angle_to_goal) + np.pi - self.theta
						elif angle_to_goal > 0 and self.theta < 0:
								actual_angle = -(np.pi - angle_to_goal + np.pi - abs(self.theta))
						else:
								actual_angle = angle_to_goal - self.theta
					else:
							actual_angle = angle_to_goal - self.theta
					
					#Calculate distance
					dist = np.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
					
					#Set speeds for different positive angles
					if abs(dist) >= 0.1 and actual_angle > 0:
						if actual_angle < 0.1:
							self.speed.linear.x = 0.25
						elif actual_angle > 1.2:
							self.speed.linear.x = 0
							self.speed.angular.z = 0.25
						elif actual_angle > 0.6 and actual_angle <= 1.2:
							self.speed.linear.x = 0.10
							self.speed.angular.z = 0.25
						elif actual_angle < 0.6 and actual_angle >= 0.2:
							self.speed.linear.x = 0.15
							self.speed.angular.z = 0.25
						elif actual_angle < 0.2 and actual_angle >= 0.1:
							self.speed.linear.x = 0.20
							self.speed.angular.z = 0.15
							
							
					#Set speeds for diffent negative angles		
					elif abs(dist) >= 0.1 and actual_angle < 0:
						if actual_angle > -0.1:
							self.speed.linear.x = 0.25
						elif actual_angle < -1.2:
							self.speed.linear.x = 0
							self.speed.angular.z = -0.25
						elif actual_angle < -0.6 and actual_angle >= -1.2:
							self.speed.linear.x = 0.10
							self.speed.angular.z = -0.25
						elif actual_angle > -0.6 and actual_angle <= -0.2:
							self.speed.linear.x = 0.15
							self.speed.angular.z = -0.25
						elif actual_angle > -0.2 and actual_angle <= -0.1:
							self.speed.linear.x = 0.20
							self.speed.angular.z = -0.15
							
					elif abs(dist) < 0.1:
						self.ready = True
						rospy.loginfo("Waypoint reached")
						self.prev_waypoint = goal_pose
						#rospy.loginfo("previous waypoint is: " + str(self.prev_waypoint))
						self.r.sleep()
							
						return self.ready	

					self.pub.publish(self.speed)
					self.r.sleep()
						
		def goal(self, goals):
				goal_poses = goals.data				
				goal_poses = np.array(goal_poses)
				
				
				#Make the even numbers the x coordinate and the odd the y coordinate
				for i in range(goal_poses.shape[0]//2):
						goal_poses_sub = np.array([goal_poses[2*i],goal_poses[2*i+1]])
						rospy.loginfo("Goal poses are the following: " + str(goal_poses_sub))
						motion_control.run(goal_poses_sub)
						if self.ready == True:
							#rospy.loginfo("READY is true")
							if i == goal_poses[-1]:
								self.last_waypoint = True
								
							i += 1

		def callbackFSM(self, state):
				self.current_state = state.data
				
				#Run the goal function only if the robot is in sweeping or low battery state

				if self.current_state == "sweeping_state" or self.current_state == "low_battery_state":
						# self.subwaypoints = rospy.Subscriber("global_waypoints", numpy_msg(Floats), self.goal)
						self.refined_waypoints = rospy.Subscriber("refined_waypoints", Float32MultiArray, self.goal)
				else:
						stop = Twist()
						stop.linear.x = 0.0
						stop.angular.z = 0.0
						self.pub.publish(stop)

if __name__ == '__main__':
		rospy.init_node('motion_control', anonymous=True)
		rospy.loginfo("Motion_control initialized")
		
		#Running the listener function of the motion_control class
		motion_control = Motion_Control()
		motion_control.listener()
