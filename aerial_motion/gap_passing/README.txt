aerial_trasformation for hydrus3
1. hydrus3 bringup launch file
roslaunch hydrus_transform_control hydrus3.launch
2. hydrus3 aerial transformation planning and control
roslaunch hydrus_gap_passing motion_control_from_file.launch
a. local pc: mocap for hydrus
roslaunch aerial_robot_base mocap.launch
b. joy stick 
roslaunch aerial_robot_base joy_stick.launch
