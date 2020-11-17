# 3D-plantModeling-with-2DLiDAR
A simulation tool for plant reconstructions based on a 2d laser sensor and a rotation disk.

<p align='center'>
    <img src="./images/doc/demo_01.gif" alt="drawing" width="400"/>
</p>

<p align='center'>
    <img src="./images/doc/Maiz_gazebo.png" alt="drawing" width="200"/>
    <img src="./images/doc/Maiz_gazebo2.png" alt="drawing" width="200"/>
    <img src="./images/doc/Trigo_gazebo.png" alt="drawing" width="200"/>
    <img src="./images/doc/Maiz_R3Dcolor.png" alt="drawing" width="200"/>
</p>

## Software requirements
* [Ubuntu 16.04](https://releases.ubuntu.com/16.04/)
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [Gazebo 7](http://gazebosim.org/download)
* Python libraries:
    * ```sudo apt-get install python-pip```
    * ```pip install numpy```
    * ```pip install pandas```

## How to install?
Clone the repository:
```
git clone https://github.com/HaroldMurcia/3D-plantModeling-with-2DLiDAR.git
```
Acces the folder:
```
cd 3D-plantModeling-with-2DLiDAR
```
Compile with catkin:
```
catkin_make
```


## How to run?

Acces the folder:
```
cd 3D-plantModeling-with-2DLiDAR
```
Add setup file:
```
source devel/setup.sh
```
Run rosmaster:
```
roscore
```
In a new terminal:
```
roslaunch plataforma plataforma.launcher
```
* Select a gazebo object
* Click on run
```
cd 3D-plantModeling-with-2DLiDAR/src
python Scaner3D_simulation.py <filename> <Max_Degree> <Delta_degree>
```
