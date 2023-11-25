# Introduction
In this repository, the [ROS](https://www.ros.org) package `assignment_1` has been implemented to satisfy the requirements of the first assignment of the course [Experimental Robotics Laboratory](https://corsi.unige.it/en/off.f/2023/ins/66551?codcla=10635) of [Robotics Engineering](https://corsi.unige.it/en/corsi/10635) course by [University degli Studi di Genova](https://unige.it).  
The assignment depends on the [aruco](https://github.com/pal-robotics/aruco_ros/tree/noetic-devel/aruco) package for acquiring and parsing the image from the camera. The robot is a [Husarion ROSbot 2R](https://husarion.com/#robots) and its model is provided by the package [rosbot_description](https://github.com/husarion/rosbot_ros/tree/noetic/src/rosbot_description). Both packages, `aruco` and `rosbot_description`, are included in this repository for convenience.  
The requirements for the assignment are the following:
 - The robot starts at coordinates (0,0) in a given environment with 4 markers with IDs 11, 12, 13 and 15.  
 The markers have the following meaning:  
    - Marker 11: rotate until you find marker 12; then reach marker 12
    - Marker 12: rotate until you find marker 12; then reach marker 13  
    - Marker 13: rotate until you find marker 12; then reach marker 15  
    - Marker 15: done!  

"reach marker XXX" means that the XXX marker must be at least 200 pixels in the camera frame.  
Implement the assignment both in simulation (the world file aruco_assignment.world is given) and with the real robot.  
In simulation, differently than with the real robot, do the "search" task only by rotating the camera, without rotating the whole robot.  
The requirements have been fulfilled as follows:
- branch [rosbot_sim](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/rosbot_sim) (from [luk1897](https://github.com/luk1897)/Experimental_Robotics-Assignment_1)  
implements the code for the simulation of the real rosbot. The architecture is based on a node implementing the controller and a node implementing the vision data handling,  
- branch [rosbot_real](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/rosbot_real) (from [luk1897](https://github.com/luk1897)/Experimental_Robotics-Assignment_1/tree/assignment-1_real_robot)  
adapts the code in branch [rosbot_sim](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/rosbot_sim) into the code loaded into the real rosbot to perform the given task,  
- branch [action_server](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/action_server)  
implements the code for the simulation of the real rosbot. The architecture is based on action server, with a node controller, providing the server, a node logic, acting as client, and a node for handling the data coming from the camera,  
- branch [action_server_rot_camera](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/action_server_rot_camera)  
implements the code for the simulation of the rosbot with rotating camera exploiting the action server controller architecture developed in branch [action_server](https://github.com/davideCaligola/experimentalRoboticsLab_assignment1/tree/action_server).  
