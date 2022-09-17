#!/usr/bin/env python

import os
import rospy
import arm_movements.error_encountered as error_movements
from std_msgs.msg import String
from std_msgs.msg import Bool
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from datetime import datetime
import smach
import smach_ros


arm_movement = error_movements.arm_error_movements()
soundhandle = SoundClient()
error_message_pub = rospy.Publisher('/human_robot_interaction_node/error_message', String, queue_size=1)
error_pub = rospy.Publisher('/human_robot_interaction_node/error', Bool, queue_size=1)
fsm_pub = rospy.Publisher('/human_robot_interaction_node/FSM', String, queue_size=1)



is_error = False
low_battery = False
is_charged = False
is_at_charging_station = False
is_finished = False
error_log = []

startup_state_fsm = "startup_state"
sweeping_state_fsm = "sweeping_state"
low_battery_state_fsm = "low_battery_state"
charging_state_fsm = "charging_state"
error_state_fsm = "error_state"
finished_state_fsm = "finished_state"



def write_to_error_log(data):

    with open(os.path.dirname(__file__) + "/error_logs.txt", "a") as f:
        f.write(data + "\n")
        f.close()


def error_callback(message):
    """This is the callback function for the error messages"""

    rospy.loginfo('HRI Error Message Received') 

    # declaring global variables in the function in order to change them
    global is_error
    global error_log
    
    # Check if error is resolved
    if message.data == "Resolved": 
        # error has been resolved
        is_error = False
        error_log.append(message.data)
        write_to_error_log(error_log[-1])

        # Use audio sound to inform the human of the robot's intention to move the arm
        soundhandle.say("Moving arm to default position", "voice_kal_diphone", 1.0)
        rospy.sleep(3)

        # Move the arm back to the Tucked position
        try:
            count = 0
            arm_movement.move_arm_to_pose("Tucked")
            rospy.sleep(5)
        except Exception as e:
            if count == 0:
                arm_movement.move_arm_to_pose("Tucked")
                rospy.sleep(5)
                count = 1
            else:
                rospy.logerr(e)

        # publish the error bool for the other nodes to continue.
        error_pub.publish(is_error)
        fsm_pub.publish(sweeping_state_fsm)

    # Check if human wan't to know what the error is   
    elif message.data == "Say Last Error":
        # 
        soundhandle.say(error_log[-1], "voice_kal_diphone", 1.0)
        rospy.sleep(3)
    # Error has been detected    
    else:
        # Error has been detected 
        is_error = True
        
        # publish error for other nodes to dicontinue operation
        error_pub.publish(is_error)
        fsm_pub.publish(error_state_fsm)

        # save error
        error_log.append(message.data)
        write_to_error_log(error_log[-1])
        
        # Use audio sound to inform the human of the robot's intention to move the arm
        soundhandle.say("Raising arm for help", "voice_kal_diphone", 1.0)
        rospy.sleep(3)

        # Move armm to Stand pose
        try:
            count = 0
            arm_movement.move_arm_to_pose("Stand")
            rospy.sleep(5)
        except Exception as e:
            if count == 0:
                arm_movement.move_arm_to_pose("Stand")
                rospy.sleep(5)
                count = 1
            else:
                rospy.logerr(e)

        # Use audio sound to inform the human of the robot's error and origin
        soundhandle.say(message.data, "voice_kal_diphone", 1.0)
        rospy.sleep(3)


def hri_startup_sequence():
    fsm_pub.publish(startup_state_fsm)
    rospy.sleep(3)
    # Start an error_log.txt
    today = datetime.now()
    str_today = today.strftime("%m/%d/%Y, %H:%M:%S")
    write_to_error_log("\n===>> NEW STARTUP SESSION: Date: " + str_today + " <<===\n")

    # using `soundplay_node` to play audio sound
    soundhandle.say("Moving arm to default position", "voice_kal_diphone", 1.0)
    rospy.sleep(3)

    
    try:
    # using Moveit to move the arm to the `Tucked` pose
        count = 0
        arm_movement.move_arm_to_pose("Tucked")
        rospy.sleep(5)
    except Exception as e:
        if count == 0:
            arm_movement.move_arm_to_pose("Tucked")
            rospy.sleep(5)
            count = 1
        else:
            rospy.logerr(e)
        

    # publishing the initial `False` for no errors detected yet
    error_pub.publish(False)


# define state startup state
class startup_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','error']
                                #input_keys=['sm_input'],
                                #output_keys=['sm_output']
                                )

    def execute(self, userdata):
        rospy.loginfo('Executing state "startup_state"')
        
        try:
            fsm_pub.publish(startup_state_fsm)
            rospy.sleep(3)
            hri_startup_sequence()
            rospy.loginfo('Completed "startup_state"')
            fsm_pub.publish(sweeping_state_fsm)
            #rospy.init_node('motion_control', anonymous=True)
            #rospy.loginfo("Motion_control initialized")
            #motion_control = Motion_Control()
            #motion_control.listener()
            #motion_control.start_motion()
            return 'success'
        except Exception as e:
            print (e)
            rospy.logerr(e)
            error_message = "HRI startup had a fault"
            error_message_pub.publish(error_message)
            #motion_control.end_motion()
            return 'error'
        #else:
        #    rospy.loginfo('Completed "startup_state"')
        #    fsm_pub.publish(sweeping_state_fsm)
        #    return 'success'

# define state sweeping state
class sweeping_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'finished', 'low_battery', 'error'])
        self.counter = 0

    def execute(self, userdata):
            if self.counter == 0:
                rospy.sleep(10)
                fsm_pub.publish(sweeping_state_fsm)
                self.counter += 1
            rospy.loginfo('Executing state "sweeping_state"')

            if is_finished == True:
                fsm_pub.publish(finished_state_fsm)
                return 'finished'
            elif is_error == True:
                fsm_pub.publish(error_state_fsm)
                self.counter = 0
                #motion_control.end_motion()
                return 'error'
            elif low_battery == True:
                fsm_pub.publish(low_battery_state_fsm)
                return 'low_battery'
            else:
                rospy.sleep(5)
                return 'continue'

            
         #  if self.counter < 10:
         #      self.counter += 1
         #      return 'success'
         #  else:
         #       error_message = "Sweeping state had a fault"
         #       error_message_pub.publish(error_message)
         #       self.counter = 0
         #       return 'error'

# define state error state
class error_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','resolved'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "error_state"')
        
        if is_error == True:
            rospy.sleep(5)
            return 'error'
        else:
            #fsm_pub.publish(sweeping_state_fsm)
            return 'resolved'

# define state low battery state
class low_battery_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','goal_not_reached', 'goal_reached'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "low_battery_state"')
        
        if is_error == True:
            fsm_pub.publish(error_state_fsm)
            return 'error'
        elif is_at_charging_station == False:
            rospy.sleep(5)
            return 'goal_not_reached'
        elif is_at_charging_station == True:
            return 'goal_reached'

# define state charging state
class charging_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','not_fully_charged', 'fully_charged'])

    def execute(self, userdata): 
        rospy.loginfo('Executing state "charging_state"')
        global is_error
        if is_error == True:
            return 'error'
        elif is_charged == False:
            rospy.sleep(60)
            return 'not_fully_charged'
        elif is_charged == True:
            return 'fully_charged'

        
def hri_main():
    rospy.sleep(15)
    # Initialising `human_robot_interaction_node`
    rospy.init_node('human_robot_interaction_node')
    # Sending an info message to terminal
    rospy.loginfo("Initialised hri_fsm_node")

    #fsm_pub.publish(startup_state)

    # Creating a subscriber to observe the error messages from all nodes
    rospy.Subscriber('/human_robot_interaction_node/error_message', String, error_callback)

     # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished_state']) 

    sis = smach_ros.IntrospectionServer('FSM_Server', sm, '/Lely_Husky')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('startup_state', startup_state(), 
                               transitions={'success':'sweeping_state', 
                                            'error':'error_state'})
        smach.StateMachine.add('sweeping_state', sweeping_state(), 
                               transitions={'finished':'finished_state', 
                                            'error':'error_state',
                                            'low_battery': 'low_battery_state',
                                            'continue': 'sweeping_state'})
        smach.StateMachine.add('error_state', error_state(), 
                               transitions={'error':'error_state',
                                            'resolved': 'sweeping_state'})
        smach.StateMachine.add('low_battery_state', low_battery_state(), 
                               transitions={'error':'error_state',
                                            'goal_not_reached': 'low_battery_state',
                                            'goal_reached': 'charging_state'})
        smach.StateMachine.add('charging_state', charging_state(), 
                               transitions={'error':'error_state',
                                            'not_fully_charged': 'charging_state',
                                            'fully_charged': 'sweeping_state'})

    
    # Execute SMACH plan
    outcome = sm.execute()
    
    
    # Infinite loop untill callback function is used.
    rospy.spin()
    


if __name__ == '__main__':
    hri_main()