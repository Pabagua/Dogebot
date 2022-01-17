# DOGEBOT
This repository contains the package to run the Dogebot task in ROS Noetic. 

## Steps to launch the Dogebot
Download the program and the latest OpenCV 2 version available, then follow the steps ahead:
* Compile the package as always with the commands:
```
$ catkin_make
$ source devel/setup.bash
```
* To bring up a gazebo world, launch the gazebo node by running:  
```
$ roslaunch dogebot dogebot_world.launch 
```
 * Once the gazebo simulation environment is launched and the robot, cube and ball are spawned, run the following to start the task on another terminal:  
 ```
$ python final_dogebot2.py 
```

## Videos of the task:
https://drive.google.com/file/d/1gs7-4im_MkW2dV3Y05aC4Y9455RHdiw8/view?usp=sharing
https://drive.google.com/file/d/1TCOK90w3twzu6sUDgn-NA7pt9p7Ol2Gd/view?usp=sharing


