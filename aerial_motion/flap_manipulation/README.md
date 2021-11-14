# Flap manipulation by end-effector (+ openning squeezing)

## Phase:
- phase0: idle (hovering)
- phase1: approach to the contact point on the flap
- phase2: contact with the flap
- phase3: move the flap
- pahse4: reset to the form before squeezing
- phase5: plan and pass through the openning  (option)

## Usage

### simulation

- bottom manipulation

```
$ roslaunch flap_manipulation dragon_bringup.launch simulation:=true real_machine:=false headless:=false
$ roslaunch flap_manipulation flap_manipulation.launch simulation:=true external_wrench_flag:=false
```

- lateral manipulation

```
$ roslaunch flap_manipulation dragon_bringup.launch simulation:=true real_machine:=false headless:=false spawn_y:=-0.6 spawn_yaw:=-1.5708 takeoff:=1.1
$ roslaunch flap_manipulation flap_manipulation.launch simulation:=true external_wrench_flag:=false
```

#### command 
- start command: click `/dragon/move_start` from rviz
- return home: rostopic pub -1 /dragon/return std_msgs/Empty "{}"

#### paramters for `flap_manipulation.launch`:

- `simulation`: spawn gazebo model of flap in gazebo
- `external_wrench_flag`: flag to enable/disable external wrench for manipulation
- `auto_state_machine`: flag to enable/disable auto shift motion state phase. When disable, please send `/dragon/move_start` in rviz every phase end.

### real machine

- bottom manipulation

```
$ roslaunch flap_manipulation dragon_bringup.launch
$ roslaunch flap_manipulation flap_manipulation.launch headless:=true
$ roslaunch flap_manipulation ground_station.launch # local PC
```

- lateral manipulation

```
$ roslaunch flap_manipulation dragon_bringup.launch takeoff:=1.1
$ roslaunch flap_manipulation flap_manipulation.launch headless:=true
$ roslaunch flap_manipulation ground_station.launch # local PC
```

#### command

- start command: PS4 joy controller`L1`
- return home command: PS4 joy controller`R1`








