# Artist-Ros
## Goal
Our goal was to create a parallel arm manipulator mechanism, with 5DOF.
We modeled the robot arm using ROS environment and RViz for the visualization. 
The folder /src/urdf contains the arm description, written in xacro, meaning a XML file that contains some macros.

## Project specifications
The project was implemented using ROS-melodic, with the use of C++ and XML.

## Execute
To execute our code do the following steps:
1. `cd catkin_ws/src`
2. `git clone <this_repo>`
3. `cd ..`
4. `catkin_make`
5. `source devel/setup.bash`
6. `roslaunch artist rviz.launch`

## Screenshots 
### Our model
<img src= "/media/Screenshot from 2019-06-25 14-59-29.png"/>

### URDF Description
<img src= "/media/artist.jpg"/>

## Future goals
Trajectory creation and script simulation.

## Contributors
<a href="https://github.com/AchilleasAlvaroHysi">
    Achilleas Alvaro Hysi 
</a> and <a href="https://github.com/anast95">
    Anastasis Tzinieris
</a>

## Disclaimer
This project was created for the undergraduate course MYE031-Robotics with professor K. Vlachos in Univercity of Ioannina.
Spring 2019.
