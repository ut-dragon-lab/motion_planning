# Samples

### 1. End-effector IK

#### Hydrus
1. start ``` $ roscore```
2.  create the environment obstacle. following command will create a simple cylinder in the environment.
```$ rosrun differential_kinematics simple_collision.py _position_x:=0.3 _position_y:=0.3 _scale_x:=0.1 _scale_y:=0.1 _scale_z:=0.3```

3. launch the IK solver
``` $ roslaunch differential_kinematics end_effector_ik.launch headless:=false ```

4. send the IK target point to this solver via rosservice.
```
$ rosservice call /end_effector_ik "target_pos: {x: -0.6, y: 0.6, z: 0.0}
target_rot: {x: 0.0, y: 0.0, z: 3.14}
orientation: true
full_body: true
collision_avoidance: true
debug: false"
 ```

*Note*: You should confirm the animation in rviz, which the end-effector (attached at the 4th link) moves to the target point [-0.6, 0.6] and yaw: -3.14 gradually without colliding into the red cylinder

### 2. Gap (Opening) Passing:

move the code to [squeeze_navigation](https://github.com/tongtybj/motion_planning/tree/master/aerial_motion/squeeze_navigation)
