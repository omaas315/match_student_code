# Redundacy Resolution for mobile Manipulator
## Overview
In the `Overview`-chapter you describe the content of this folder, either in text or a listing. For example:

This folder acts as an example for all the future students that will upload their code to this repository. I contains an example ROS-package that only contains a `README`-file to show how a package should be documented. Also any additional Matlab- or Python-script for the analysis of data should be added here.

**Author:** Hauke Heeren

**E-Mail:** hauke.heeren@stud.uni-hannover.de

## Packages
### optimization_algo 
Uses an Optimization to calculate the velocity of the MiR. UR velocity is cartesian velocity command minus the MiR velocity. 

## Scripts
### optimize.py 
- input: cartesian velocity command
- output: UR velocity
- output: MiR velocity

Can either be used as a class to be implemented in own script, or otherwise by just running the main.
Needs a running controller for the Robot arm listening to topic at `cooperative_manipulation/cartesian_velocity_command`.
