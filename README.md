[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Home Service Robot

5th project for Robotics Software Engineer Nanodegree @Udacity

## Objective
Robot has to navigate on its own to a specific pose and picks up a virtual marker.
Later it has to navigate to destination pose and drops off the virtual marker.

## Packages
"home_service_robot" project consists of several custom built along with pre-existing packages from ROS community.

Localization is achieved using AMCL algorithm.
Environment Mapping is done by 'gmapping' package.
To achieve Robot Navigation, 'Dijkstra's algorithm' (a variant of the Uniform Cost Search algorithm) is used. Which results in, ROS navigation stack creates a path for the robot while avoiding obstacles on its path.
'pick_objects' node - Here multiple destinations will be provided to robot.
'add_marker' node - Subscribes to the destinations topic published by previous nodes and takes care of visualization of marker.

The Gazebo world design intends to replicate the **WorldSkills Mobile Robotics** next challange in a simplified version.
For more information please take a look on a brief competition overview at the end.

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
