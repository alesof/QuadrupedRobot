#Quadruped Robot Control & Navigation
[inserire immagine]

##Technical Project
This package allows to simulate the control of a quadruped robot and the locomotion in an unkown environment. 

##Install
- Install Dependencies

$ sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev ros-noetic-gmapping
$ sudo apt install libopencv-dev python3-opencv


- Install IPOPT

$ git clone https://github.com/ethz-adrl/ifopt.git
$ cd ifopt
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install


- Build Towr

$ git clone https://github.com/ethz-adrl/towr.git
$ cd towr/towr
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make
$ sudo make install

- Install IDynTree
IDynTree Library is also needed to run this package. To install and build it follow the steps described in: [iDynTree](https://github.com/robotology/idyntree/blob/master/doc/build-from-source.md "iDynTree")

Finally to install the package clone this repo in your catkin workspace and then build it with `catkin_make -DCMAKE_BUILD_TYPE=Release`

## Run
Run the simulation using `roslaunch project project.launch`.

Then start the controller using `rosrun project professore_node`.

Press play in gazebo to start the simulation.
