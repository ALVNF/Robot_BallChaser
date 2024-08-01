# Robot Ball Chaser

## Introduction
This is a small robotics initiation project, with the intention of improving my skills in this field.
Using technologies such as **ROS** and **Gazebo Classic**.

## Setup
In order to run this project, you need to install previously [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on Linux preferably on **Ubuntu 20.04**.

Once you are done with the installation, you need to create a workspace with a /src directory.
```console
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
```
After that, clone this repository inside the /src folder.
```console
$ cd src/
$ git clone https://github.com/ALVNF/Robot_BallChaser.git
```
Finally you should be able to build the packages.
```console
$ cd ..
$ catkin_make
```
## Run Project
For starting the program, use the following command inside the catkin_ws folder
```console
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

On another terminal, start the next command also inside the catkin_ws folder
```console
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
After running the ball_chaser.launch, the robot will look for a white point and if it finds it, it will start chasing it.

If you want to see the robot's point of view, open a new terminal also inside the catkin_ws folder
```console
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view
```
Make sure that the image path is **/camera/rgb/image_raw**