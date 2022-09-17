## 1. Human Robot Interaction Node --> `human_robot_interaction_node`
The purpose of this node is to allow a human to safely and effectively interact with the Husky robot. The only instance in which we would like a human to interfere with the robot's duties is if there is an error with the robot that can only be solved by a human, or if the robot is having difficulties performing it's task - for example, it can't reach a narrow area which the path planner has intructed it to go. 

Another useful tool that this node provides to the human is a web observation page that can live stream what the robot is currently doing via it's camera, it displays error messages, and it can also display the route that the robot has already sweeped. This can be useful if the farmer is not constantly in the barn house, but would like to still supervise the robot.

### 1.1 Main Node Functions
In order for this node to perform it's duties, the functionalities have been split into four distinct functions.
- Move Arm
- Error Audio
- Teleoperation
- Observation


![HRI Node Functions](README_images/HRI_functions.png)


If an error is detected the robot will stop all operations and lift it's hand to signal for help. It will also play an audio sound detailing when it is about to lift the hand and inform the human where the error originated from. The error audio is useful for a human to know what the robot will be doing next.

The web observation page is useful for the farmer to keep an eye on the robot from a distance. It also provides a Teleoperation function for the farmer to manually drive the robot around in the event that the planner or robot dynamics have failed.

### 1.2 Detailed Node implementation
The Human Robot Interaction Node is included in the "husky_lely.launch" file, so it starts automatically when the simulation is run. The node structure is as follows:
- Starts by initialising the 'human_robot_interaction_node'. 
- Then it initialises the 'soundplay_node'.
- Then it initalises a rosbridge_websocket for the web observer.
- Tucks the arm away.
- Subscribes to the error message topic.
- Waits for an error.

In order to be able to use the soundplay_node, please download the following dependencies:

```
$   rosdep install sound_play
$   rosmake sound_play
```

In order to see which speakers are available and to change the default speakers, please see the following link:
source: http://wiki.ros.org/sound_play/Tutorials/ConfiguringAndUsingSpeakers

In order to use the rosbridge_websocket, please download the following dependencies:

```
$   sudo apt-get install ros-<rosdistro>-rosbridge-suite
```

#### Move Arm
Since the arm movements will be very simple, `Moveit` is used to move the arm from one pose to another. During normal operation and at startup the arm should not get in the way, therefore a new `Tucked` pose has been created. If an error occurs, a moveit_commander object has it's target pose set to `Stand`. The moveit_commander will then plan and execute the instruction, and wait until the instruction has been completed. See the two images below showing the "Tucked" and "Stand" poses respectively.

![Tucked](README_images/Tucked.png)                                                                   ![Stand](README_images/Stand.png)

Error messages are published to a topic with the following structure: 
- topic: `/human_robot_interaction_node/error_message`
- datatype: `std_msgs/String` 

Messages are structured as follows:
```
{
  "data": "This is my error message to the human"
}
```

Each node can publish an error message to this topic, and since the `human_robot_interaction_node` is subscribed to this topic, it will automatically "raise the alarm" that an error has occured, stop all operations, and raise it's hand to signal for help.

If the error message contains the following message:
```
{
  "data": "Default Position"
}
```
Then the error has been resolved and the arm will move to the `Tucked` pose, signalling that the robot is ready to start it's normal operations again.

#### Error Audio
The messages published to `/human_robot_interaction_node/error_message` are also converted to audio using the text to speech conversion. These audible sounds are simply an added safety function for the human to be aware of the robot's intentions and the error without having to run detailed diagnostics.

The soundplay_node was not implemented by the group, instead it was sourced from the link provided in the "Detailed Node Implementation".

#### Teleoperation and Observation
This functionality makes use of a web app called `Foxglove Studio`, which uses all the information published to `/rosout`, and can use this information to create a user interface best suited for the farmer's needs. See image attached:

![Web App](README_images/web_app.png) 

In order to use this application, simply navigate to:

source: https://studio.foxglove.dev/?ds=rosbridge-websocket&ds.url=ws%3A%2F%2Flocalhost%3A9090&layoutId=d10e79dc-ac63-4628-8c80-099d1e3a4160

if this doesn't work, open the following web page:

source: https://foxglove.dev/studio

Select 'Open Web App', 'Open Connection', 'Rosbridge (ROS 1 & 2)' and connect to websocket URL: 'ws://localhost:9090'.

The human can use the control interface to teleoperate the robot and also watch the robot's live movements on the web app.

### 1.3 Node Conclusion
Hopefully this documentation gave a good overview of the funtionalities of this node, and why it is useful in this project. 
