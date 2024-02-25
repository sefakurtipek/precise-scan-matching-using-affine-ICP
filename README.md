# Affine ICP with Correntropy Criterion for Laser Scan Matching
## Overview
The `dock_scan_matching` package is a specialized ROS package, it implements the affine Iterative Closest Point (ICP) algorithm with correntropy criterion and point-to-line metric, based on the paper "A Precise Scan Matching Based Localization Method for an Autonomously Guided Vehicle in Smart Factories". It is tailored for accurate docking tasks in a factory.  Entropy (correntropy) criterion is used in order to provide robustness against noise and/or outliers.
![affineICP2](https://github.com/sefakurtipek/precise-scan-matching-using-affine-ICP/assets/36789388/83bb056d-085b-420a-b2b2-11a7b7f809be)
![affineICP1](https://github.com/sefakurtipek/precise-scan-matching-using-affine-ICP/assets/36789388/b143d0ca-c345-4197-9de4-2e5fdb8d4c0a)
## Dependencies
- ROS (tested on Melodic)
- PCL (tested on 1.2)
- Eigen3
- C++11 or later

## Installation
```bash
git clone git@github.com:sefakurtipek/precise-scan-matching-using-affine-ICP.git
catkin_make // to make build
```
## Usage

```bash
rosbag play scanMatchBag1.bag
```
The user needs to write the following commands on linux terminal to take parameters and run ROS package

```bash
roslaunch dock_scan_matching params.launch
```
In order to visualize I used rviz. I need to remap tf and tf_static according to name of the robot. For example, I took "/r300311695" name from 'rostopic list' command

```bash
rosrun rviz rviz /tf:=/r300311695/tf /tf_static=/r300311695/tf_static
```

## Parameters you need to know
The user may need to tune given parameters located in `params.launch` file.
`initial_guess_X`: Guess point x value for target charge station center point according to map frame\
`initial_guess_Y`: Guess point y value for target charge station center point according to map frame\
`initialGuess_yaw_degrees`: Guess point yaw degree angle for target charge station center point according to map frame\
`maxIteration`: Maximum iteration for affine ICP algorithm to find closest point matching \
`epsilon`: Threshold value for affine ICP\
`vShapeModelName`: robot charge station model point cloud file. Type of file is `.pcd`\
`laserFrameName`: Frame name of laser scan data\
`mapFrameName`: Map frame name. It is used to get frame transformation\
`laserScanName`: Laser scan data topic name. It is used to subscribe laser scan data\     
`absolute_path`: Absolute path of project files. It is used to give model point cloud file location\
```bash
<param name="initial_guess_X" type="double" value="-8.2" />     
<param name="initial_guess_Y" type="double" value="-7.2" />
<param name="initialGuess_yaw_degrees" type="double" value="265.364" />
<param name="maxIteration" type="int" value="5" />
<param name="epsilon" type="double" value="0.2" />
<param name="vShapeModelName" type="string" value="RMRS004.pcd" />
<param name="laserFrameName" type="string" value="/r300311695/base_front_laser_link" />
<param name="mapFrameName" type="string" value="v/mjdtwfb31e2e/map" />
<param name="absolute_path" type="string" value="/home/sefa/catkin_ws/src/dock_scan_matching/src/" />
```
