<p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="https://i.imgur.com/6wj0hh6.jpg" alt="Project logo"></a>
</p>

<h3 align="center">Optimization of mur movement</h3>

<div align="center">

[![Status](https://img.shields.io/badge/status-active-success.svg)]()
[![GitHub Issues](https://img.shields.io/github/issues/pumablattlaus/optimization_algo.svg)](https://github.com/pumablattlaus/optimization_algo/issues)
[![GitHub Pull Requests](https://img.shields.io/github/issues-pr/pumablattlaus/optimization_algo.svg)](https://github.com/pumablattlaus/optimization_algo/pulls)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> Optimization for murs with impedance control
    <br> 
</p>

## 📝 Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Problem solving](#problems)
- [Usage](#usage)
- [TODO](./TODO.md)

## 🧐 About <a name = "about"></a>

Optimization for murs with impedance control in regard to cartesian ur-path

## 🏁 Getting Started <a name = "getting_started"></a>

put project in catkin folder

### Prerequisites

hrl_kdl noetic-dev branch (nur für pykdl_utils.kdl_kinematics.joint_list_to_kdl und pykdl_utils.kdl_parser.kdl_tree_from_urdf_model benötigt)
```
git clone -b noetic_dev https://github.com/pumablattlaus/hrl-kdl.git
cd hrl-kdl/pykdl_utils
pip3 install .
cd ..
cd hrl_geom
pip3 install .

#install numdifftools:
pip3 install numdifftools
```
PyKDL muss installiert sein

timed_roslaunch, match_lib, robotiq for simulation

```
git clone ...
```


## 🔧 Problem solving <a name = "problems"></a>

Explain how to run the automated tests for this system.

### Break down into end to end tests

Admittance Controller movement: Make sure vel_command is at 100 Hz (not that importatnt anymore because time is measured)

```
rostopic hz /mur/ur/joint_group_vel_controller/command
```
Same for optimization Node:
```
rostopic hz /mur/cooperative_manipulation/cartesian_velocity_command
```

mur has been at amcl branch before
### And coding style tests

Explain what these tests test and why

```
Give an example
```

## 🎈 Usage <a name="usage"></a>

To start in simulation:
```
roslaunch gazebo_ros empty_world.launch
roslaunch optimization_algo spawn_murs_obj_fixed.launch
roslaunch optimization_algo grasp_and_control.launch
```

To use for existing robot:
```
roslaunch optimization_algo murs_control.launch
```

One robot in hardware (mur):
```
# On mur:
roslaunch mur_launch_hardware mur.launch
roslaunch optimization_algo control.launch launch_optimization:=false

#on rosmater:
roslaunch optimization_algo control.launch launch_admittance:=false

# cmds:
rostopic pub /mur/velocity_command geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0 
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```