#!/usr/bin/env python

import os
import rospy
import arm_movements.error_encountered as error_movements
from std_msgs.msg import String
from std_msgs.msg import Bool
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from datetime import datetime
#from FSM.states import *

arm_movement = error_movements.arm_error_movements()
soundhandle = SoundClient()
error_pub = rospy.Publisher('/human_robot_interaction_node/error', Bool, queue_size=1)
fsm_pub = rospy.Publisher('/human_robot_interaction_node/FSM', String, queue_size=1)
is_error = False
error_log = []

startup_state = "startup_state"
sweeping_state = "sweeping_state"
low_battery_state = "low_battery_state"
charging_state = "charging_state"
error_state = "error_state"

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
        arm_movement.move_arm_to_pose("Tucked")
        rospy.sleep(1)

        # publish the error bool for the other nodes to continue.
        error_pub.publish(is_error)
        fsm_pub.publish(sweeping_state)

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
        fsm_pub.publish(error_state)

        # save error
        error_log.append(message.data)
        write_to_error_log(error_log[-1])
        
        # Use audio sound to inform the human of the robot's intention to move the arm
        soundhandle.say("Raising arm for help", "voice_kal_diphone", 1.0)
        rospy.sleep(3)

        # Move armm to Stand pose
        arm_movement.move_arm_to_pose("Stand")
        rospy.sleep(1)

        # Use audio sound to inform the human of the robot's error and origin
        soundhandle.say(message.data, "voice_kal_diphone", 1.0)
        rospy.sleep(3)

        
def main():
    
    # Initialising `human_robot_interaction_node`
    rospy.init_node('human_robot_interaction_node')
    # Sending an info message to terminal
    rospy.loginfo("Initialised human_robot_interaction_node")
    
    fsm_pub.publish(startup_state)

    # Start an error_log.txt
    today = datetime.now()
    str_today = today.strftime("%m/%d/%Y, %H:%M:%S")
    write_to_error_log("\n===>> NEW STARTUP SESSION: Date: " + str_today + " <<===\n")

    # using `soundplay_node` to play audio sound
    soundhandle.say("Moving arm to default position", "voice_kal_diphone", 1.0)
    rospy.sleep(3)

    # using Moveit to move the arm to the `Tucked` pose
    arm_movement.move_arm_to_pose("Tucked")
    rospy.sleep(5)

    # publishing the initial `False` for no errors detected yet
    error_pub.publish(False)

    fsm_pub.publish(calibration_state)


    
    # Creating a subscriber to observe the error messages from all nodes
    rospy.Subscriber('/human_robot_interaction_node/error_message', String, error_callback)
    
    # Infinite loop untill callback function is used.
    rospy.spin()
    


if __name__ == '__main__':
    main()