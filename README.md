# Selfie @ Carolo-Cup 2019

## Gazebo install instructions

follow the instructions on this page: http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros

If you have already installed Gazebo, make sure that you have version 7. Otherwise nothing will work, and you will need to delete gazebo and install version 7.


## Launching simulator
 Build everything and run ./source/setup.bash

If you installed everything properly the only thing you need to do is run:

roslaunch selfie_race selfie_sim.Launching

Be patient.. it opens quite slowly :(
This will open gazebo and rviz windows.
Simulator publishes /scan and camera/zed/rgb/image_rect_color
To be able to control selfie you have to publish something on /keyboard_steering_ackermann/teleop topic.
(name will be changed to more convenient one in the future)

You can also control the car with the keyboard by running:
rosrun selfie_control keyboard_teleop.py


Good luck.
