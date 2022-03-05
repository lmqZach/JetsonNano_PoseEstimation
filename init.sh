roscore &
sleep 5
rosrun jetbot_ros jetbot_camera &
sleep 1
rosrun jetbot_ros jetbot_control.py &
sleep 1
rosrun jetbot_ros camera_info_publisher.py &
sleep 1
ROS_NAMESPACE=jetbot_camera rosrun image_proc image_proc &
sleep 2
roslaunch navigation_dev navigation.launch &