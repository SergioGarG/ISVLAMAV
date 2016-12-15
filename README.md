# ISVLAMAV
Indoor SLAM using Vision and Laser for Micro Aerial Robots


ISVLAMAV is a project that allows low-cost MAVs to perform SLAM in real-scale environments. The system can be implemented over the following hardware platforms:

1. ErleCopter

The system use information from the monocular camera and the IMU that are implemented in most of the commercial drones. For this reason, the system needs of a monocular VSLAM algorithm to be run. Any algorithm could be implemented remapping the topics, but we have used:

1. LSD-SLAM: https://github.com/tum-vision/lsd_slam
2. ORB-SLAM: https://github.com/raulmur/ORB_SLAM

Our project needs also of an algorithm that returns the position of the drone in 3 DoF. We use:

1. Hector_slam: https://github.com/tu-darmstadt-ros-pkg/hector_slam

# Prerequisites

1. ROS
We have tested the system in Ubuntu 12.04 with ROS Hydro and in Ubuntu 14.04 with ROS Indigo.

2. Visual SLAM algorithm
Any VSLAM method which brings the 6 DoF drone's position could be used. The system will recognice automatically LSD-SLAM and ORB-SLAM.

3. Laser SLAM algorithm
Our project needs also of an algorithm that returns the position of the drone in 3 DoF.

4. Eigen
The project uses Eigen for the calculations.
http://eigen.tuxfamily.org/index.php?title=Main_Page

5. A hardware platform that can carry a Lidar and that has enough computational capacity to process the algorithms
For instance, the Erle Copter uses a Raspberry 2B.

# Installation

Just go to your workspace project, type catkin_make and press enter on your keyboard.

# Usage

1. Set the desired path for your drone in pid_main_retardos.cpp. 
2. Launch your VSLAM algorithm.
3. Launch your Laser SLAM algorithm.
4. Launch the EKF node.
rosrun ekf ekf_islamav

5. Make your MAV to take off and launch the PID controller node.
rosrun PID pid_main_retardos_islamav
