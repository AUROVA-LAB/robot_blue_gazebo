# Gazebo simulator ROS packages for BLUE robot

### Requirements to use this package:

- System requirements: Ubuntu 20.04, ROS Noetic, and Gazebo 11.

- Install package for GPS and IMU (download kinetic branch): https://github.com/tu-darmstadt-ros-pkg/hector_gazebo

- Install package for Velodyne: https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/

- Install controllers packages: 
```shell
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

- Install the package with plugins for Ackermann actuators: https://github.com/trainman419/ackermann_vehicle-1

- Download the gazebo models from https://github.com/osrf/gazebo_models and move them to the system folder:
```shell
sudo mv /PATH_TO_gazebo_models/* /usr/share/gazebo-11/models
```

- Install this package.
```shell
catkin_make --only-pkg-with-deps robot_blue_gazebo
```

### Example of usage:

You can run an example following the instructions in [applications](https://github.com/AUROVA-LAB/applications) (Example 2).

### Gazebo plugins
In this package, there is a gazebo plugin to create animated actors which stop when there is an obstacle in front of them. Here is an example of use:

```
<actor name="actor">
    <skin>
        <filename>moonwalk.dae</filename>
    </skin>
    <animation name="walking">
        <filename>walk.dae</filename>
    </animation>
    <animation name="moonwalk">
        <filename>moonwalk.dae</filename>
    </animation>

    <plugin name="actor0_plugin" filename="libActorStopObstaclePlugin.so">

        <stop_distance>1</stop_distance>
        <animation_factor>3</animation_factor>
        <obstacles>
            <model>
                ackermann_vehicle
                <max_x>1.32</max_x>
                <min_x>-0.27</min_x>
                <max_y>0.475</max_y>
                <min_y>-0.475</min_y>
            </model>
        </obstacles>

        <trajectory id="0" type="walking">
            <waypoint>
                <time>0</time>
                <pose>2 0 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>2</time>
                <pose>-2 0 0 0 0 -1.57</pose>
            </waypoint>
        </trajectory>
        <trajectory id="1" type="moonwalk">
            <waypoint>
                <time>0</time>
                <pose>-2 0 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>6</time>
                <pose>2 0 0 0 0 -1.57</pose>
            </waypoint>
        </trajectory>

    </plugin>
</actor>
```
More information about the skins and animations available can be found at https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot. About this plugin, the actor stops when the obstacle is at *stop_distance* meters in front of it. The *animation_factor* is used to synchronize the animation with the movement. The *obstacle* tag is used for defining the obstacles, in this case the robot blue. Each model is defined by its name and four parameters to define the collision rectangle of the model with respect to its position (in this case it is in the center of the rear wheels and not in the center of the model). Finally, you define the trajectory of the actor. You can define multiple trajectories sorted by its id, using different animations. Each trajectory is defined by a sequence of waypoints
