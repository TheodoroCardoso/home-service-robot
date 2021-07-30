[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Home Service Robot

5th project for Robotics Software Engineer Nanodegree @Udacity

## Objective
Robot has to navigate on its own to a specific pose and picks up a virtual marker.
Later it has to navigate to destination pose and drops off the virtual marker.

## Packages
Home Service Robot project consists of several custom built along with pre-existing packages from ROS community.

Localization is achieved using AMCL algorithm, which is a probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach using a particle filter to track the pose of a robot against a known map.


Environment Mapping is done by 'gmapping' package. This package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, it is possible create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.


ROS navigation stack creates a route for the robot while avoiding obstacles on its path. It takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base. 'Dijkstra's algorithm' (a variant of the Uniform Cost Search algorithm) is used to compute the trajectory through an obstacle-free path. 

'pick_objects' node - Here multiple destinations will be provided to robot.

'add_marker' node - Subscribes to the destinations topic published by previous nodes and takes care of visualization of marker.

## How to Launch the simulation?

#### Create a catkin_ws, feel free to skip if you already have one!
```sh
$ cd /home/workspace/
$ mkdir -p /home/workspace/catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
$ cd ..
```

#### Clone the package in catkin_ws/src/
```sh
$ cd /home/workspace/catkin_ws/src/
$ git clone https://github.com/TheodoroCardoso/home-service-robot.git
```

#### Build the packages
```sh
$ cd /home/workspace/catkin_ws/ 
$ catkin_make
```

#### Once packages have been built, you can launch the bash script that will take care of everything else
```sh
$ ./src/scripts/home_service.sh
```

#### That's it! Now just watch the robot moving the virtual marker from the pick up zone to the drop off zones.
