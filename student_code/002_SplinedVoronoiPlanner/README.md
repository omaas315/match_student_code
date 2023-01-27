# Splined Voronoi Planner
## Overview
In the `Overview`-chapter you describe the content of this folder, either in text or a listing. For example:

This folder acts as an example for all the future students that will upload their code to this repository. It contains an example ROS-package that only contains a `README`-file to show how a package should be documented. Also any additional Matlab- or Python-script for the analysis of data should be added here.

**Author:** Sönke Prophet

**E-Mail:** soenke.prophet@gmail.com

For recreation of thesis results you have to copy the bagfiles to data folder; they are not contained in git due to large filesize.


## Packages
### example_pkg 
Explain in one sentence what the package is. In the package should be another README that deeply explains how to use the package. See the example file in the `example_pkg`-folder for the structure.

### match_path_planning/splined_voronoi
Global Path planning and smoothing based on voronoi diagrams. Code is already contained in match_path_planning and therefore only contained as git submodule.

## pure_voronoi_planner
Global path planning on voronoi diagrams without smoothing. Used for comparison between dijkstra and astar and planning with and without additional freespaces.

### splined_voronoi_scripts
A package which contains scripts for generating evaluation data and creating plots.
For a better overview over the scripts and how to call them see [README](splined_voronoi_scripts/README.md)

## Scripts
### example_script.py 
Explain what the script does and how to use it. Also mention any inputs and outputs.

## data
Contains bagfiles used to generate results. 
The data is stored separately due to large filesize.

Pltos will also be saved in this folder.
