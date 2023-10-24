# Robust Trajectory Tracking for Quadrotor with Sliding Mode Control
Design a sliding mode controller for altitude and attitude control of Crazyflie 2.0 drone to track desired trajectories between desired waypoints. The controller should be robust to external disturbances. (Implemented as part of project for RBE502 - Robot Control course)

## Steps to setup the workspace and run the code
- For this project Ubuntu 20.04 with ROS Noetic is used.
- Install additional ROS dependencies to set up Crazyflie 2.0 in Gazebo
  ```
  sudo apt update
  sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
  sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
  sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler
  libgoogle-glog-dev
  rosdep update
  sudo apt-get install ros-noetic-ros libgoogle-glog-dev
  ```
  
- Create a ROS workspace and download the packages for the drone.
  
  ```
  mkdir -p ~/myworkspace/src
  cd ~/myworkspace/src
  catkin_init_workspace # initialize your catkin workspace
  cd ~/myworkspace
  catkin init
  cd ~/myworkspace/src
  git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
  git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
  ```

- For the other data change the variables accordingly and run the file.
- To generate 3D animations uncomment the specified lines in 'main' function. 
- In Code folder:
  ```
  python Wrapper.py
  ```
## Report
For detailed description of the math see the report [here](Report.pdf).
## Plots and Animations
For the train data 1, plots and animation showing roll, pitch, and yaw for all the filters:
<!---
<p float="middle">
<img src="outputs/p1a.png" width="750" height="450"/>
<img src="outputs/p1b.png" width="750" height="450"/>
</p>
<p float="middle">
<img src="outputs/output1.gif" width="750" height="350"/>
</p>
------>

Remaining plots are present in the report and links to rest of the animations are 


## References
1. [https://github.com/gsilano/CrazyS](https://github.com/gsilano/CrazyS)
2. [https://www.bitcraze.io/products/old-products/crazyflie-2-0/](https://www.bitcraze.io/products/old-products/crazyflie-2-0/)

## Collaborators
Chaitanya Sriram Gaddipati - cgaddipati@wpi.edu

Anoushka Baidya - abaidya@wpi.edu
  
