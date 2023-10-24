# Robust Trajectory Tracking for Quadrotor with Sliding Mode Control
Design a sliding mode controller for altitude and attitude control of Crazyflie 2.0 drone to track desired trajectories between desired waypoints. The controller should be robust to external disturbances. (Implemented as part of project for RBE502 - Robot Control course)

## Steps to run the code
- Install Numpy, Scipy, and Matplotlib libraries before running the code.
- To run on the first training data in the `Wrapper.py` file in the 'main' function set the variables as:
	IMU_filename = 'imuRaw1' and vicon_filename = 'viconRot1'
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

## Collaborators
Chaitanya Sriram Gaddipati - cgaddipati@wpi.edu

Anoushka Baidya - abaidya@wpi.edu
  
