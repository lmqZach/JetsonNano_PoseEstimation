# AprilTag Detection and Robot Pose Estimation

## File Structure

```
|-- ROOT
  |-- README.md
  |-- CMakeLists.txt
  |-- init.sh
  |-- jetbot_ROS
  |   |-- ```
  |-- jetson-inference
  |   |-- ```
  |-- navigation_dev
  |   |-- CMakeLists.txt
  |   |-- package.xml
  |   |-- launch
  |   |-- msg
  |   |-- src
  |          |-- april_detect.py
  |          |-- localizatiojn_node.py
  |          |-- planner_node.py
  |-- ros_deep_learning
      |-- ```
```
Main algorithms reside under root/navigation_dev/src

## Objective
Calibrated onboard camera using rospy and OpenCV, for AprilTags detection and robot pose estimation. Use the on-board camera to drive to specific locations in the environment. 

## Preparation Tasks
1. AprilTags Setup from: https://github.com/AprilRobotics/apriltag
2. Camera Calibration on ROS, with instrunction from: https://github.com/peter-moran/jetson_csi_cam

## Detailed Tasks
1. Drive to position (1,0,0) place a reference object at an appropriate location to assist with localization.
2. Continue onwards to location (1,2,π) again with the assistance of an object for localization
3. Return to location (0,0,0).
4. For each position estimate your localization error

## Report
### Logistic: 
As the consecutive project from the first one, the main goal is to use the onboard camera to provide instantaneous feedback to correct the Jetbot trajectory on the ground plane, to reach target points with more accuracy. One necessary step before implementation is to calibrate the onboard camera to allow effective tag localization. Theoretically, the pose information should be the absolute robot pose in the world frame, but for tracking waypoints in this project, a relative pose with respect to April tags would be sufficient, and it is used to correct the path and plan for the next tag.
### Control Algorithm:
It takes minimal effort to complete ROS camera calibration, but our group did physically flip the camera view angle for easier computation. For the best quality of tag detection, tags are placed 0.5m from the target waypoint, on the direction of travel. Our group divided the main design logic into three parts: calculating the tag’s relative position to camera, completing a path to a specific tag with correction, and finding the next tag through rotating the Jetbot.
### Code Explanation:
Regarding ROS node setup and message transfer in matrix computation, the localization node receives ID lists of recognized April-tags and their corresponding transformation matrices. It then publishes the robot pose information to the planner node, computed as p_camera=T_tag^camera·p_tag. Since we are moving in 2D plane, we would ignore errors in the z-axis. Taking the origin of tags, we set p_tag=[0,0,0,1]^T, which essentially just makes the tag position in the camera frame identical to the translation vector in T_tag^camera. Finally, our localization node publishes a list with 3 values: a relative x position, a relative y position, and a tag ID. If no tag is detected, those 3 values will be [0, 0, -1]. 
To complete path correction, our group splits the speed of wheels into two parts: an offset turning speed called ‘diff’ proportional to ‘relative_x’ with opposite signs on each wheel, and a ‘forward’ speed proportional to ‘relative_y’. To avoid going too fast to miss the tag in the video footage, forward speed is confined under 0.4. 
The value of variable ‘relative_y’ also determines if the current waypoint is reached, since all tags are placed 0.5m from the waypoint. For locating the next tag, the code is developed to keep Jetbot rotating until the next tag is detected. Due to the lag between video and Jetbot movement, constant rotation does not guarantee a successful detection of the next tag in actual tests. As a result, discrete rotation is involved, controlled by a Boolean variable ‘turn_flag’, to allow relatively stationary phases for tag detection.
Performance:
Initially, the above code generates satisfying results at 70% - 80% chance. Failures happen in two scenarios. Our group assigned ‘forward’ speed to be linear to ‘relative_y’ to complete the movement more efficiently. However, when ‘relative_y’ is too large, the correction speed is so fast that the robot would move completely out of the camera range which can detect the current tag, which leads to a messy trajectory in the end. The other problem happens, after reaching a specific tag, that the wheel speed is too low to rotate the robot on the test surface, which would cause the robot to stay in one place in front of the previous tag.
The mentioned problems are solved by capping the forward speed to V=0.4, along with changing two-wheel differential rotation to one-wheel counterclockwise rotation at v=0.2. The engineered adjustment guarantees every successful test for the last four runs, during which, the robot will arrive at the waypoints with correct facing and return to the origin with very little deviation as shown in the example video.
### Limitations:
Most limitations in this project are brought by April tags. For instance, there is no absolute guarantee that the next detected tag is the actual ‘next’ one, if the camera somehow skips the second tag and jumps to the third one. The other limitation is hidden in the environment setup. Since each tag is attached to a booklet and humanly placed on the ground, the distance to waypoint, degree of facing, and flatness of the tag surface have a great variation to the industrial standard. Theoretically, when moving to the target tag, the Jetbot may detect other tags at the very edge of video footage, which could cause incorrect trajectory. 
The other drawback is the energy consumption in completing the movement. Our algorithm satisfies the trajectory requirement but requires discrete motion in turning, which uses a lot of power to complete. 
### Potential Improvements:
The limitations in tag detection are theoretically possible, but our group did not encounter related problems in real tests. Our idea of solving them is by a sorted queue of tag_IDs. It will make sure the Jetbot’s ability to detect the correct number of tags in an engineered sequence. This solution can be easily implemented, if necessary, in our future projects.
Potential improvement can be made through hardware updates to achieve better energy utilization. A better camera will lead to higher frame rate of the video, and a more advanced motor structure will lead to a slower but more controllable turning speed. Both hardware changes will lead to the possibility of consistent motion, which is more power-efficient.

## Performance Video Link:
Complete Run: https://youtu.be/YYee9Y_hJ8U 



