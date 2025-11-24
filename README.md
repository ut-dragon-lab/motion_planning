# motion_planning
Motion planning for aerial robot and other robot

## dependency
- jsk_aeiral_robot:
   - url: [https://github.com/jsk-ros-pkg/jsk_aerial_robot](https://github.com/jsk-ros-pkg/jsk_aerial_robot)
   - version:
        - &gt; [1.3.6](https://github.com/jsk-ros-pkg/jsk_aerial_robot/tree/1.3.6)
        - recommendation: [6ac557b5](https://github.com/jsk-ros-pkg/jsk_aerial_robot/tree/6ac557b562e4078d8a203f4a04857a68c078f171)

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

## how to compile

```
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash # Please make sure the path to repository of jsk_aerial_robot
mkdir -p ~/ros/motion_planning_ws
cd ~/ros/motion_planning_ws
wstool init src
wstool set -u -t src motion_planning http://github.com/tongtybj/motion_planning --git
wstool merge -t src src/motion_planning/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```
