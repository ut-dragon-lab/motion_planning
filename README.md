# motion_planning
Motion planning for aerial robot and other robot

## Dependency
- jsk_aerial_robot (verison 1.3.6):

  https://github.com/jsk-ros-pkg/jsk_aerial_robot/tree/1.3.6

  This project is compatible with jsk_aerial_robot 1.3.3 to 1.3.6.

- For Ubuntu 22.04 with ROS-O, perform both of the following steps:

  - For `fcl_catkin` , add the supplementary apt repository:

    https://github.com/ut-dragon-lab/ros-o-overlay/blob/main-jammy-one/README.md

  - For `libfcl.so.0.6`, manually install `fcl` from source:

    ```bash
    git clone --branch v0.6.1 --single-branch https://github.com/flexible-collision-library/fcl.git
    cd fcl
    mkdir build && cd build
    cmake ..
    sudo make install
    sudo ldconfig
    ```


## Packages
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

## How to compile

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash # Please make sure the path to repository of jsk_aerial_robot
mkdir -p ~/ros/motion_planning_ws
cd ~/ros/motion_planning_ws
wstool init src
wstool set -u -t src motion_planning https://github.com/ut-dragon-lab/motion_planning.git --git
wstool merge -t src src/motion_planning/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```
