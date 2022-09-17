## Perception node
The perception node is included in the "husky_lely.launch" file so it starts automatically when the simulation is run. The node structure is as follows:
- Subribes to raw top-view camera image as input: **"/camera_1/image_raw"**. 
- Processes input using edge detection methods to segment image. Output is published to **"/image_process/output_image"**. 
- Segments image and creates occupancy map. This is also the occupancy map to be used by the path planner. Each pixel that is an obstacle gets the value **'255'** while the others are set to **'0'**. This is then published to **"/image_process/occupancy_map"**. 
- Looks for Aruco marker in image to localize the robot. The coordinates of the robots are then published to **"/image_process/husky_location"**. The visual location is then also published as an annotated image to **"/image_process/husky_tracking"**
- Localized all moving obstacles in draws circle around them. Output is then published to **"/image_process/moving_obstacles"**

**Output message types and encodings:**
- Occupancy map: **<sensor_msgs>** with encoding **"TYPE_MONO8"** 
- Husky visual location: **<sensor_msgs>** with encoding **"BGR8"** 
- Moving obstacle location: **<sensor_msgs>** with encoding **"BGR8"** 
- Huksy coordinates: **<geometry_msgs>** with type **"Point"** 


With the current image encoding RVIZ is not able to show the output images properly so the rqt viewer can be used instead as follows:

``` bash
source devel/setup.bash
rqt_image_view
```

In the top menu select the correct publish topic according to the ones mentioned above. 

An example of the image outputs can be seen below:

**Input Image**:
![input image](README_images/input_image.png)

**Segmented Image**
![segmented image](README_images/occ_map.png)

**Husky location**
![husky location](README_images/husky_track.png)

**Moving obstacles**
![moving obstacles](README_images/obstacles.png)
