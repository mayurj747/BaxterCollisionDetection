# BaxterCollisionDetection
Performs collision detection in python for Baxter 

## Requires: 
* ROS (tested on Kinetic, Ubuntu 16.04 LTS) 
* everything in here: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation#Baxter_Simulator_Installation
* MoveIt

Demo-ing collision detection capabilities of the Baxter robot in python. 

_state_validity_collision.py_ contains the StateValidity class which is responsible for collision detection

_set_environment.py_ is a helper script which will create an environment complete with collision objects for demo purposes. 

## Optional
Getting contact information in case of collision is optional. If not required, comment out the relevant lines (marked in code) in _state_validity_collsion.py_

Publishing contact information requires a custom msg type - _ContactInformationArray_. 

_baxter_tests_ is a catkin package that contains this message type. To install, navigate to ```<root_wkspace>/src/``` , place _baxter_tests_ here, cd back to ```<root_wkspace>``` and ```catkin_make```.

_collision_publisher.py_ is an optional script that runs in the background (keeping the publisher alive) and re-publishes ```/robot_collision_contacts``` with latching on so that contact information can be viewed through ```rostopic echo``` even after the main python script has terminated. 


