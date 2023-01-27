# Splined Voronoi Planner
## Overview

This folder contains all code for the work on a global path planner on voronoi diagrams with smoothing.
It contains submodules to relevant packages and should be contained in an ROS noetic catkin workspace for building and using the packages.
It also contains packages for evaluating and generting examples.

**Author:** Sönke Prophet

**E-Mail:** soenke.prophet@gmail.com

The data of the evaluation is stored on an USB-drive because of large filesize.
To reproduce the results please copy the files to the data folder.
Also copy relevant maps to splined_voronoi_scripts/maps.


## Packages

### match_path_planning/splined_voronoi
Global Path planning and smoothing based on voronoi diagrams. Code is already contained in match_path_planning and therefore only contained as git submodule.

Further instructions for installation are in the contained README.

### pure_voronoi_planner
Global path planning on voronoi diagrams without smoothing. Used for comparison between dijkstra and astar and planning with and without additional freespaces.

### splined_voronoi_scripts
A package which contains scripts for generating evaluation data and creating plots.
For a better overview over the scripts and how to call them see [README](splined_voronoi_scripts/README.md)

### match_mobile_robotics
Package with general packages for mobile robotics at the match institute. It is only contained as a dependency for easier building.

## data
Contains bagfiles used to generate results. 
The data is stored separately due to large filesize.

Pltos will also be saved in this folder.
