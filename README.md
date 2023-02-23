# alfarobi_neo_ws
ROS workspace for Alfarobi's humanoid robots development. The following workspace will work perfectly fine on Ubuntu 20.04 (ROS Noetic).


## 1. Installation Guide
After installing ROS Noetic, the following dependencies are required.

* Make sure everything is up-to-date
```
sudo apt-get update
sudo apt-get upgrade
```

* For working with multiple C++ libraries
```
sudo apt-get install build-essential
sudo apt-get install gcc-multilib g++-multilib
```

* Build the C++ Dynamixel SDK library. This is mandatory for Dynamixel port handling. We usually create a directory called "program/" inside the workspace to store required programs/dependencies. You may use the following command:
```
# clone the Dynamixel SDK repository inside program directory
cd alfarobi_neo_ws && mkdir program && cd program
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

# make the port handler
cd alfarobi_neo_ws/program/DynamixelSDK/c++/build/linux64 && make
sudo make install
sudo chmod a+rw /dev/ttyUSB0
```
