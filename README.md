# motion_planning
Motion planning for aerial robot and other robot

## aerial_manipulation
Manipulation planning for aerial robot.
- aerial_transportation: whole-body grasping to pick up object and carry to a desired position

## aerial_motion
Aerial transformation planning 
- gap_passing: gap passing by transformation
```
$ roslaunch gap_passing gap_passing_planning.launch
```
```
<simulator>: simulation or real machine
<play_log>: play offline planning log or online planning
<record_log>: save online planning log or discard
<replay_experiment_data>: replay rosbag data mode or not
```
- real machine with offline planning log
```
$ roslaunch gap_passing gap_passing_planning.launch simulator:=false play_log:=true
```
- simulation with offline planning log
```
$ roslaunch gap_passing gap_passing_planning.launch simulator:=true play_log:=true
```
- simulation with online planning and save plannning result
```
$ roslaunch gap_passing gap_passing_planning.launch simulator:=true play_log:=false record_log:=true
```
- simulation replay rosbag data
```
$ roslaunch gap_passing gap_passing_planning.launch simulator:=true play_log:=true replay_experiment_data:=true
```


## navigation

General 2D motion planning 
