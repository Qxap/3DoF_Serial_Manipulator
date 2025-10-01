# 3DoF_Serial_Manipulator
This repository contains the necesary code to run a short simulation in RViz of a R-R-P Robot

# Steps to run the simulation
In order to successfully run the simulation, first make sure to change the path of the `dxf_waypoints_v5.csv` file in the `trajectory_planner_node` for yours (line 19), as well as the output path for the `planned_trajectory.csv` (line 197).

Go to the `dxf_exporter_node_v2`, change the `traj_shape_amg.dxf` default location for yours (line 35) Do the same procedure for `dxf_waypoints_v5.csv` at line 170.

Remember not to use the relative paths.


execute `ros2 launch ms2r1p robot_launch_ms2r.launch.py`; the stationary robot should be displayed in RViz.
Then run the planner trajectory node using `ros2 run ms2r1p trajectory_planner_node`, the robot must start moving following the trajectory that follows the edge of the .stl figure.
