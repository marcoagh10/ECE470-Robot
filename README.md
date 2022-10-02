# ECE470-Robot
This repository will be where our group stores our code and other files for the ECE 470 project.

:robot:

Team name: The Bot-lers

Team members:
- Marco Guzman (marcog2)
- Marius Juston (mjuston2)
- Beno Maya (bmaya2)


Robot Butler - Canned Drinks

The team has decided to be innovative college students and create a personal robot butler that can bring individuals canned drinks. The goal of the project is to create a robot that has the capabilities of navigating through a small environment (i.e. an apartment), reaching into a target container (i.e. fridge) in order to open it, find the target canned drink, grab it from the container and then bring it back, while keeping the can stable and upright to the target individual. Our group plans to accomplish this task by using the robot’s gripper attachment and through the Gazebo simulator using ROS. Additionally, in order to accomplish this task the robot will be aided in the use of camera sensors in order to find and identify the cans in the container, a 2D LIDAR in order to avoid obstacles on the floor during navigation, and contact sensors to properly verify that the cans have been collected.

Installation Steps:
- git clone https://github.com/marcoagh10/ECE470-Robot
- git submodule update --init --recursive
- cd catkin_botlers
- catkin_make
- source devel/setup.bash
- rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
- roslaunch gazebo_world small-room-ur3.launch
