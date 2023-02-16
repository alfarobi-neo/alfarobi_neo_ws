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

* Build the C++ Dynamixel SDK library and copy it to root folder. This is necessary for Dynamixel port handling.
```
cd alfarobi_neo_ws/src/alfarobi_dxlsdk/src/c++/build/linux64 && make
sudo make install
sudo chmod a+rw /dev/ttyUSB0
```