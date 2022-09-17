#!/usr/bin/env python

from socket import MsgFlag
import os
import rospy
import smach
import smach_ros
from hri_fsm_node import *

# define state startup
class startup_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','error']
                                #input_keys=['sm_input'],
                                #output_keys=['sm_output']
                                )
        

    def execute(self, userdata):
        rospy.loginfo('Executing state "startup_state"')
        try:
            hri_startup_sequence()
        except Exception as e:
            print (e)
            rospy.logerr(e)
            error_message = "HRI startup had a fault"
            error_message_pub.publish(error_message)
            return 'error'
        else:
            rospy.loginfo('Completed state "startup_state"')
            return 'success'

# define state sweeping
class sweeping_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'error'])
        self.counter = 0

    def execute(self, userdata):
           rospy.loginfo('Executing state "sweeping_state"')
           if self.counter < 3:
               self.counter += 1
               return 'success'
           else:
                error_message = "sweeping_state had a fault"
                error_message_pub.publish(error_message)
                self.counter = 0
                return 'error'

class error_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','resolved'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "error_state"')
        global is_error
        if is_error == True:
            return 'error'
        else:
            return 'resolved'

class low_battery_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','goal_not_reached', 'goal_reached'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "error_state"')
        global is_error
        if is_error == True:
            return 'error'
        else:
            return 'resolved'

class charging_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','not_fully_charged', 'fully_charged'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "error_state"')
        global is_error
        if is_error == True:
            return 'error'
        else :
            return 'resolved'


# main
def main():
    rospy.init_node('states')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('startup_state', startup_state(), 
                               transitions={'success':'sweeping_state', 
                                            'error':'error_state'})

        smach.StateMachine.add('sweeping_state', sweeping_state(), 
                               transitions={'success':'sweeping_state', 
                                            'error':'error_state'})
        smach.StateMachine.add('error_state', error_state(), 
                               transitions={'error':'error_state',
                                            'resolved': 'sweeping_state'})
        smach.StateMachine.add('low_battery_state', error_state(), 
                               transitions={'error':'error_state',
                                            'goal_not_reached': 'low_battery_state',
                                            'goal_reached': 'charging_state'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()