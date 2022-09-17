# Obstacle Avoidance as Navigation

> This node is contributed and maintained by _Liangchen Sui (l.sui@student.tudelft.nl)_. Any suggestions and critics are welcomed ; )


## Guidance

- [1. General Description](#1-general-description)

- [2. File Structure](#2-file-structure)

- [3. How to Use](#3-how-to-use)

- [4. Demo](#4-demo)

- [5. Future Work](#5-future-work)

---
## 1. General Description

- ### Overview

    This repository constructs a subsidiary package for the Lely project based on ROS as **_Navigation Module_**. The main challenge here is to provide an accurate and safe algorithmatic solution for obstacle avoidance **under highly dymamic and random environment**.

- ### Methodology and Improvements
    
    The algorithm is based on [Artificial Potential Field](http://www.diag.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf "Click to see the reference") (APF)${}^{[1]}$ in workspace $W$. The implemented method aims to overcome its incomplete nature caused by local minimum with a workaround of **_online, discritized best-first algorithm_** under dynamic environment.

    For the attractive field, the following is used, where $q_{goal} \subset W_{goal}$.
    
    $$
    \begin{align}
    U_a(q) = \frac{1}{2} K_a (q_{goal} - q)^T (q_{goal} - q)    
    \end{align}
    $$

    For the repulsive field, the following is used, where $q_i \subset W_{obstacle}$.
    
    $$
    \begin{align}
    U_r(q)_i    & = \left\{
    \begin{align*}
    & \frac{1}{2} K_r (\frac{1}{q_{i}} - \frac{1}{q})^2 &, q_i < \delta \\
    & 0 &, q_i > \delta \\
    \end{align*}
    \right. \\

    U_r(q)      & = \sum_{i=1}^N U_r(q)_i
    \end{align}
    $$
- ### Information Flow

    ```mermaid
    flowchart LR
        perception==>|occupancy_map|obstacle_avoidance
        perception==>|robot_position|obstacle_avoidance
        global_planning==>|global_waypoints|obstacle_avoidance
        obstacle_avoidance==>|refined_waypoints|motion_control
        HRI==>|FSM|obstacle_avoidance
        obstacle_avoidance==>|error|HRI
    ```

    Given `occupancy_map`, `robot_position`, `global_waypoints` and `FSM_state`, this node generates an APF considering both static and dynamic obstacles and output `refined_waypoint` and `error` message in a real-time manner.

---
## 2. File Structure

The file tree under this package `obstacle_avoidance` is structured as:

```
.
├── apf_avoider
│   ├── apf_avoider.py
│   ├── apf_multi_avoider.py
│   ├── apf_real_avoider.py
│   └── apf_velocity_avoider.py
|
├── fake_map
│   ├── fake_map.py
│   ├── lely_map.PNG
│   └── lely_map.yaml
|
├── fake_waypoints
│   └── fake_waypoints.py
|
├── launch
│   ├── demo.launch
│   └── obstacle_avoidance.launch
|
├── CMakeLists.txt
├── package.xml
└── README.md
```

- As a collection of distinctively implemented variations of APF, the folder `apf_avoider` contains 4 following Python scripts:

  - the `apf_avoider.py` for a naive implementation
  - the `apf_multi_avoider.py` for better optimality with multiple repulsive field generated
  - the `apf_velocity_avoider.py` for velocity-based motion control
  - the `apf_real_avoider.py` for cooperative work after calibration

  ```
   .
   └── apf_avoider
      ├── apf_avoider.py
      ├── apf_multi_avoider.py
      ├── apf_real_avoider.py
      └── apf_velocity_avoider.py
  ```

- For demo simulation and performence test, the folder `fake_map` and `fake_waypoints` constructs a self-made simulatior with more  randomly moving obstacles under user's customization.

    ```
    .
    ├── fake_map
    │   ├── fake_map.py
    │   ├── lely_map.PNG
    │   └── lely_map.yaml
    |
    └── fake_waypoints
        └── fake_waypoints.py
    ```

- For ROS compilation and launch, 

    ```
    .
    ├── launch
    │   ├── demo.launch
    │   └── obstacle_avoidance.launch
    |
    ├── CMakeLists.txt
    └── package.xml
    ```

---
## 3. How to Use

- To begin with, please firstly go back to the parent directory in your workspace and build the catkin workspace for ROS with command `catkin build` or `catkin_make`.

- For a sole demo with built-in mini simulator, please run this command in the terminal:
    ```
    roslaunch obstacle_avoidance demo
    ```

- For an integrated project within Gazebo, please run this command in the terminal:
    ```
    roslaunch cor_husky_gazebo husky_lely
    ```

---
## 4. Demo

As shown below, the demo is operated in a mini simulator with multiple obstacles moving randomly. The monitor contains 5 scopes for detailed visualizations:

- Attractive Field
- Repulsive Field
- Artifitial Potential Field (Sum)
- Local Scope
- Global Scope

![alt text](obstacle_avoidance/apf_final_demo.gif)

_*The red rectangle circumscribes the local perceptive field, where the APF is actually generated._

_*The blue dot is located at the center of robot._

_*The cyan dot is the look-ahead refined waypoint to follow for real-time obstacle avoidance._

_*The green dot is the final goal to reach._


---
## 5. Future Work${}^{[2]}$

- To tune hyperparamters {`movingStep`, `attractiveScaler`, `repulsiveScaler`} for a more suitable step and a better safety margin.

- To mitigate the inevitable loss of acuraccy when robot approaching the wall and being repelled away.

- To erase accumulated errors through light-weighted implementation with better real-time performance.

---
## Reference

[[1]](http://www.diag.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf "Click to see the reference") Giuseppe Oriolo, _Motion Planning: Artificial potential fields_, Autonomous and Mobile Robotics (AMR), Sapienza Università di Roma

[[2]](https://asl.ethz.ch/publications-and-sources/publications.html?batch_name=publications&page=0) Roland Siegwart, Illah Reza Nourbakhsh and Davide Scaramuzza, _Introduction to Autonomous Mobile Robots_. the MIT Press
