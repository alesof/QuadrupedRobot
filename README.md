# Quadruped Robot Control & Navigation
This repo contains the package needed for the RL and FSR modules final exam. Media folder contains a pdf describing the project and a video of a complete simulation. The video is modified to run at 4x speed.

Table of Contents
=================

  * [Technical Project](#Technical-Project)
  * [Install](#Install)
  * [Run](#Run)
    * [Additional Settings](#Additional-Settings)
    * [QR Code Generator](#QR-Code-Generator)

## Technical Project
This package allows to simulate the control of a quadruped robot and the locomotion in an unkown environment. The last stable version is tested on Ubuntu 20.04 OS, using ROS Noetic and Gazebo 11.5.1.

This package also contains:
- A modified version of the [DogBotV4](https://github.com/ReactRobotics/DogBotV4 "DogBotV4") by ReactRobotics
- A modified version of the [Towr](https://github.com/ethz-adrl/towr "towr") project, extended to use the DogBotV4 model
- [Alglib](https://www.alglib.net "Alglib") library needed for the controller quadratic problem
- A modified version of [QRCodeGenerator](https://github.com/RaymiiOrg/cpp-qr-to-png "QRCodeGenerator"), used to generate png files for the QR Markers
- A scene reproducing a home-like environment
- A gazebo plugin to test external forces effects on the robot


## Install
- Install Dependencies
```
$ sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev ros-noetic-gmapping libzbar-dev
$ sudo apt install libopencv-dev python3-opencv
```

- Build IPOPT
```
$ git clone https://github.com/ethz-adrl/ifopt.git
$ cd ifopt
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

- Build Towr
```
$ git clone https://github.com/ethz-adrl/towr.git
$ cd towr/towr
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make
$ sudo make install
```
- Install IDynTree
IDynTree Library is also needed to run this package. To install and build it follow the steps described in: [iDynTree](https://github.com/robotology/idyntree/blob/master/doc/build-from-source.md "iDynTree")

Finally to install the package clone this repo in your catkin workspace and then build it with `catkin_make -DCMAKE_BUILD_TYPE=Release`

## Run
To run the simulation follow this steps in order:

1. Run the simulation using `roslaunch project project.launch`

2. Start the controller using `rosrun project apf`

3. Press play in gazebo to start the simulation

------------


### Additional Settings
- **Change Room Coordinates :** if you're using a different simulation scene and need to change the rooms' coordinates, values are defined in the service_server.cpp file. Changing these values also may result in exploring the given scene in a different room order. 
- **Change Marker ID :** to change the ID the robot has to find modify the variable _marker_desiderato defined in the globalplanner.cpp file. Follows a list of all the encrypted ID's.

| Room  | ID | 
| :---: | :---: |
| Room1 | ID1234 |
| Room2 | ID2997 |
| Room3 | ID0101 |

------------

### Qr Code Generation
To use the QR Code generator library in the package first you need to build it with the following instructions.

```
cd project/cpp-qr-to-png-master
mkdir build
cd build
cmake ..
sudo make all
```

Then generate a QR Code as a png file with
```
cd project/cpp-qr-to-png-master/build/src
sudo ./main
```
You'll then be prompted to enter a string to encrypt as QR, and a name for the file. **Note** that the file name has to end with the ".svg" extension and there's no input validation loop.
