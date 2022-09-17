# Path planning node
This nodes takes care of calculating a feasible path for husky to cover the entire map in the quickest way possible.
As input it take the occupancy map and position published by the perception node.
As output it publishes an array of 2D coordinates to follow by Husky to fully cover the map.

## Structure of the node
The node is made up out of a single class, ```PathPlanning```. The inputs of the class are onlt the occupancy map, pixel size of the robot and the localization of Husky. These input together are used to generate waypoints over the entire map. These waypoints are solved using a custom first solution heuristic. Also another solver is there, called ```google_tsp()```, but our handmade solution create a much more optimal path. 

Once the object is initialized the following function will iniate the whole path planning node:
```python
path_planning = PathPlanning(
        ros_enabled = ros_enabled,
        husky_size = husky_size,
        start_position_husky = start_position_husky,
        geometric_distance = geometric_distance,
        waypoint_scale = 2, 
        solver = solver
    )

path_planning.listener()
```

## Installing OR-tools
In order to run this node, a dependency needs to be installed, the optimization library from Google and scikit-learn.
```bash
pip3 install ortools scikit-learn
```

## Expected results
The result in the whole application should be the publish of messages which contain the entire path array on the rostopic ```/global_waypoints```. This should look something like this:
```bash
layout: 
  dim: []
  data_offset: 0.0
data: 
  - [1.4 9.8]
  - [ 2.1 15.4]
  - [ 2.8 15.4]
  - [ 3.5 15.4]
  - [ 4.2 15.4]
  - [ 4.9 15.4]
...

```

The output of the line graph should look something like this:

![Line plot path graph](images/path_list.jpeg?raw=true "Path")

## Launching the nodes
The node itself can simply be launched by launching the launch file
```bash
roslaunch path_planning path_planning.launch
```

