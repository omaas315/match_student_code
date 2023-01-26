## splined voronoi scripts

This is a package containing scripts useful for analyzing the splined voronoi planner.

It is used for:
- generating random plans
- feeding plans in planner and save results in bagfiles
- generating example plots
- read out stats from bagfiles and perform analyzation
- plotting of data

First create a venv inside of package for installing requirements:
```bash
cd splined_voronoi_scripts
python3 -m venv venv
source venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

For generating results run scripts from 002_SplinedVoronoiPlanner folder.

ToInclude:
- src for generating different sized voronoi?


## spline_helpers.py
contains functions which are useful for spline generation and analyzing which are used in various different scripts such as:
- calc_connection_points: calculates connection points from a given set of waypoints
- calc_control_points_for_path: calculates control points for quintic bézier spline from waypoints and length of the tangents
- calc_control_points_for_path_sra: calculates control points for quintic bézier spline by method used in splined relaxed a star. This uses waypoint positions as connection points.
- get_coefficient_matrices: creates bernstein polynomials for each bézier curve in bézier spline
- get_sample: get value of an bézier curve based on the bernstein polynomials and a fraction (position on curve between 0 and 1)
- sample_spline_path: get samples from control points of bézier spline
- SplineAnalyzer: class which gets waypoints, tangent lengths and map from bagfile and calculates info an spline and distance to obstacles 

## path_helpers.py
contains functions which are useful for path handling
- points_from_path: type conversion from ROS-Message type to numpy array
- calc_angles: calculates orientation of each point in path from direction to next point
- calc_derivative: calculates numerical derivative of discrete samples
- offset_path: for a path with angles apply a constant offset to get single robot location
- world_to_map: transform 2d points from metric world frame to pixel based map frame


## bagfile_helpers.py
contains function for loading data from bagfiles
- get_contents_from_bagfile: assumes single data per tompic and saved them in dictionary


## splining_example.py
creates plots of an example quintic bézier-spline
- path_smooth_old: splined relaxed astar version of quintic spline generation from waypoints and consequences of tangent length variation
- path_smooth: my generation of connection points between waypoints and corresponding tangents
- path_smooth_vars: effect of tangent length on bézier spline
- path_smooth_opt: optimal choice of tangent length for good aproximation of initial path

also creates plot for single quintic bézier curve
- single_quintic_bezier_curve: bézier curve with control points an dtangents for visualization

## bagfile_examples.py
creates plot for example splining and optimizing process
- sparsing_example_from_bag: creates plot for exaple sparse path
- splining_exmple_from_bag: creates plot for example splined and optimized path

## single_formation_path.py
analyzes single path as example for planning success
- plots path of two robots of formation with 1m radius
- plot distance of formation center to obstacles to show collision avoidance
- plot orientation and anglular velocities of robots to show path is feasible with nonholonomic robots


## compare_algorithms.py
prints quota and time for each optimization algorithm

## compare_sra_and_voronoi.py

- calculate and plot distances to obstacles for every successful path
- compare achieved curvature and path length
- print time and success quota
