# 3DoF_Serial_Manipulator
This repository contains the necesary code to run a short simulation in RViz of a R-R-P Robot
The code is quite self-explanatory. In order to successfully run the simulation, first, execute `ros2 launch ms2r1p robot_launch_ms2r.launch.py`; the stationary robot should be displayed in RViz.
Then run the planner trajectory node using `ros2 run ms2r1p trajectory_planner_node`, the robot must start moving following the trajectory of the CAD piece shown below.
