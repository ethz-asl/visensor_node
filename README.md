visensor-node
=============

ROS interface to the Visual-Inertial Sensor developed by the [Autonomous Systems Lab (ASL), ETH Zurich](http://www.asl.ethz.ch) and [Skybotix AG](http://www.skybotix.com). This sensor provides fully time-synchronized and factory calibrated IMU- and stereo-camera datastreams. A detailed spec sheet of the sensor can be found [here](http://www.skybotix.com/skybotix-wordpress/wp-content/uploads/2014/03/VISensor_Factsheet_web.pdf). The low-level driver for the VI-Sensor can be found [here](https://github.com/ethz-asl/libvisensor).

![alt text](http://wiki.ros.org/vi_sensor?action=AttachFile&do=get&target=vi-sensor-front.jpg "Sensor Photo")

A detailed description on how the sensor can be used in ROS is found in the corresponding [ROS-Wiki](http://wiki.ros.org/vi_sensor).


## Installation

Check out the sensor library and this node to your catkin workspace:

```
cd your_catkin_workspace
git clone https://github.com/ethz-asl/visensor_node.git
git clone https://github.com/ethz-asl/libvisensor.git
```

Make sure that you installed all necessary ROS packages

```
sudo apt-get install libeigen3-dev libopencv-dev libboost-dev ros-indigo-cmake-modules
```
Adjust the packet name to your ros version.

Build the package using catkin_make

```
catkin_make
```
