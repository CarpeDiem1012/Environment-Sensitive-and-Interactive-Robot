#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
#from moveit_ros_planning_interface import _moveit_roscpp_initializer


class arm_error_movements():
    """Arm error movements for when an error is encountered or solved"""
    def __init__(self):

        # Initialising moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        #_moveit_roscpp_initializer.roscpp_init("human_robot_interaction_node_moveit")


        
        try:

            # Initialise MoveItInterface objects
            arm_group_name = "kinova_arm"
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
            self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                               queue_size=1)


        except Exception as e:
            print (e)
            self.initialisation_success = False
        else:
            self.initialisation_success = True


    def move_arm_to_pose(self, pose):
        arm_group = self.arm_group

        rospy.loginfo("Moving arm to pose: " + pose)
        
        # Set the target pose
        arm_group.set_named_target(pose)
        #arm_group.set_start_state_to_current_state()

        # Plan pose trajectory
        planned_pose_path = arm_group.plan()

        # Follow trajectory and wait till it has reached the goal pose
        return arm_group.execute(planned_pose_path, wait=True)

    #def move_arm_to_joint_angles(self, tolerance=0.01):
    #    arm_group = self.arm_group
    #    success = True

    #    # Get the current joint positions
    #    joint_positions = arm_group.get_current_joint_values()
    #    self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration to "Tucked Away"
    #    joint_positions[0] = 0
    #    joint_positions[1] = 2.0
    #    joint_positions[2] = 2.69
    #    joint_positions[3] = -1.6
    #    joint_positions[4] = 2.57
    #    joint_positions[5] = 1.5

    #    arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
    #    arm_group.go(wait=True)
 

#def main():
#    arm_movement = arm_error_movements()
#
#    success = arm_movement.initialisation_success
#
#    if success:
#        success &= arm_movement.move_arm_to_pose("Stand")
#    else:
#        print("Something went wrong with the initialisation of MoveItInterface objects.")
#    print (success)
#
#    #if success:
#    #    success &= arm_movement.move_arm_to_pose("Home")
#    #else:
#    #    print("Something went wrong with the initialisation of MoveItInterface objects.")
#    #print (success)
#
#    if success:
#        success &= arm_movement.move_arm_to_joint_angles()
#    else:
#        print("Something went wrong with the initialisation of MoveItInterface objects.")
#    print (success)




#if __name__ == '__main__':
#  main()
