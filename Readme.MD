# FinalSim

This is a point simulator for a drone. The original code was adapted from [FlightGoggles](https://flightgoggles.mit.edu). The simulator was optimized to require low computational overhead.

## Installing the Simulator

First, we will start by cloning the repo:
```
$ cd ~
$ git clone https://github.com/hildebrandt-carl/labsim.git
$ cd ~/labsim
```

Next, install all the prerequisites required for this project:
```
$ sudo apt-get update
$ sudo apt-get install libsdl-dev -y
$ sudo apt-get install python-pip -y
$ python2 -m pip install --upgrade pip
$ python2 -m pip install scipy
```

Now we are going to install the ROS dependent dependencies:
```
$ cd ~/labsim
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
$ sudo apt-get install ros-kinetic-joy -y
$ sudo apt-get install ros-kinetic-imu-filter-madgwick -y
$ sudo apt-get install ros-kinetic-map-server -y
```

Next, we are going to compile to the workspace.
```
$ cd ~/labsim
$ catkin build
```

## Using the Simulator

Finally, we can run the simulator
```
$ source ~/finalsim/devel/setup.bash
$ roslaunch flightcontroller fly.launch map_file:=map_medium0.yaml
```

To fly the drone around manually you can click on the keyboard gui and use the following keys:
* arrow keys -> Change X and Y
* u key -> Increase Z
* d key -> Decrease Z
* r key -> Increase yaw
* l key -> Decrease yaw
* Enter -> Send command

## Editing the maps

The following maps have been added for you to use:
* map_medium0.yaml
* map_medium1.yaml
* map_medium2.yaml
* map_large0.yaml
* map_small0.yaml

To edit the map we recommend using Pinta. To install Pinta you can use:
```
sudo add-apt-repository ppa:pinta-maintainers/pinta-stable
sudo apt-get update
sudo apt-get install pinta
```