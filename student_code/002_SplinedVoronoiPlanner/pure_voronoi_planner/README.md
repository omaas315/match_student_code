# Pure voronoi Planner

for evaluation of voronoi generation and path planning on voronoi a pure planner is introduced.


Contains:
- pure planner with dynamic reconfiguration for freespace and dijkstra vs astar comparison
- dynamicvoronoi and sweepline voronoi generation for comparison
- dynamicvoronoi algorithm taken from http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
- comparison between dynamicvoronoi and sweepline for images with different resolutions

## Starting pure voronoi planner

Can be used as plugin of move_base_flex with name pure_voronoi_planner/PureVoronoiPlanner

Example Launchfile is in [splined_voronoi_scripts/launch](../splined_voronoi_scripts/launch/pure_voronoi_planner.launch)

## Starting comparison of voronoi generation

```bash
rosrun pure_voronoi_planner compare_voronoi_results path/to/map.pgm
```

Feeds map in dynamicvoronoi and sweepline and saves results and intermediate images in data/voronoi folder.
Scales map to 4 resolutions
Saves times for generating in txt for plotting data.

