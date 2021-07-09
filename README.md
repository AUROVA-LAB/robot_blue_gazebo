# Gazebo simulator ROS packages for BLUE robot

### Requirements to use this package:

- System requirements: Ubuntu 16.04, ROS Kinetic, and Gazebo 7.

- Install Gazebo and Gazebo ROS Package: http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros

- Install package for GPS and IMU (download kinetic branch): https://github.com/tu-darmstadt-ros-pkg/hector_gazebo

- Install package for Velodyne: https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/

- Install controllers packages: 
```shell
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

- Install the package with plugins for Ackermann actuators: https://github.com/trainman419/ackermann_vehicle-1

### Example of usage:

You can run an example following the instructions in: [application_navigation](https://github.com/AUROVA-LAB/application_navigation) and [application_localization](https://github.com/AUROVA-LAB/application_localization).
