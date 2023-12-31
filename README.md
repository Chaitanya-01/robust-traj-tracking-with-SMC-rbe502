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
- Configure and Build the workspace
  ```
  cd ~/myworkspace
  rosdep install --from-paths src -i
  rosdep update
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
  catkin build
  ```
- Source the workspace
  ```
  echo "source ~/myworkspace/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ``` 
- With all dependencies ready, build the ROS package by the following commands:
  ```
  cd ~/myworkspace
  catkin build
  ```
- Check if everything works by spawning the drone in Gazebo
  ```
  roslaunch rotors_gazebo crazyflie2_without_controller.launch
  ```
- Clone this repository into `src` folder as a package and build the workspace.
- To run the code first open a terminal and spawn the Crazyflie 2.0 quadrotor on the Gazebo simulator (play the simulation)
  ```
  roslaunch rotors_gazebo crazyflie2_without_controller.launch
  ```
- Open a new terminal to run the control script. Launch the ROS node for the quadrotor to follow the trajectory:
  ```
  rosrun project quadrotor_control.py
  ```
- Once the simulation is done, the actual trajectory is saved to `log.pkl` file. To visualize go to `scripts` folder and run:
  ```
  python visualize.py
  ```
  
## Report
For detailed description of the math see the report [here](Report.pdf).
## Plots and Animations
The path followed by the drone after simulation:

<p float="middle">
<img src="project/scripts/trajectory.png" width="750" height="450"/>
</p>

<p float="middle">
<img src="project/scripts/gif.gif" width="750" height="350"/>
</p>

## References
1. [https://github.com/gsilano/CrazyS](https://github.com/gsilano/CrazyS)
2. [https://www.bitcraze.io/products/old-products/crazyflie-2-0/](https://www.bitcraze.io/products/old-products/crazyflie-2-0/)

## Collaborators
Chaitanya Sriram Gaddipati - cgaddipati@wpi.edu

Anoushka Baidya - abaidya@wpi.edu
  
