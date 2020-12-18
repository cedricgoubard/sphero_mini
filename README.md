# Sphero Mini ROS package

This simple [ROS](https://www.ros.org/) package can be used to connect and interact with [a Sphero Mini robot](https://sphero.com/products/sphero-mini).
Developed with ROS Noetic, Sphero firmware 12 and catkin tools.

We use the Python interface developed by [MProx](https://github.com/MProx/Sphero_mini).

## Installation

### Prerequisites
 - [Install ROS Noetic](http://wiki.ros.org/noetic/Installation)
 - Make sure your catkin workspace is set up (cf. [the official tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))
 - Install the catkin tools using `sudo apt install python-3catkin-tools` (or `python-catkin-tools` if you are using python 2)

### Installing the package
 - Move to the `src` folder of the workspace
 - Clone this repository
 - Move back to the root of your workspace `cd ..`
 - Build using `catkin build`

### Connecting to the robot
To interact with the robot, you need to know its MAC address. To find it, scan the available devices using `sudo hcitools lescan`.
Once you know the MAC address:
 - Create a configuration file `cp [PATH_TO_THE_PACKAGE]/config/sphero_conf.json.example [PATH_TO_THE_PACKAGE]/config/sphero_conf.json`
 - Edit the new file with your MAC address
 - Rebuild the package `catkin build` 
