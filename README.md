# motion_planning
Motion planning for aerial robot and other robot

## dependency
- aeiral_robot:
   - url: https://github.com/tongtybj/aerial_robot
   - version: [1.3.1](https://github.com/tongtybj/aerial_robot/tree/1.3.1)

## packages:
### aerial_manipulation
Manipulation planning for aerial robot.
- aerial_transportation: whole-body grasping to pick up object and carry to a desired position

### aerial_motion
#### methods
- sampling based search
- differential kinematics
#### application
- aerial transformation 
- gap passing
- object manipulation

#### navigation
General 2D motion planning 

## how to compile

```
cd <catkin_ws>
wstool merge -t src src/motion_planning/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```
