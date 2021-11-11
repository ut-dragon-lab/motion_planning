# Flap manipulation by end-effector (+ openning squeezing)

## squeezing motion with flap moving
### basic command
```
$ roslaunch squeeze_navigation dragon_bringup.launch
$ roslaunch squeeze_navigation dragon_passing_planning.launch headless:=true start_squeeze_path_from_real_state:=true teleop_flag:=true
```

### use following command to proceed the phase 
```
$ rostopic pub -1 /dragon/phase_proceed std_msgs/Empty "{}"
```
#### Phase:
- phase0: hovering
- phase1: approach to the contact point on the flap
- phase2: contact with the flap
- phase3: move the flap
- pahse4: reset to the form before squeezing
- phase5: plan and pass through the openning
