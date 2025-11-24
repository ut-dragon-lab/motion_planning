#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q curl wget sudo lsb-release gnupg git sed build-essential # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

# Install ROS
if [[ "$ROS_DISTRO" ==  "one" ]]; then
    git clone https://github.com/jsk-ros-pkg/jsk_aerial_robot.git
    cd jsk_aerial_robot && ./configure.sh
else
    sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update -qq

    if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
        sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon python-is-python3
    else
        sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
    fi
    sudo apt-get install -y -q ros-$ROS_DISTRO-catkin

    # Setup for rosdep
    sudo rosdep init
    rosdep update --include-eol-distros
fi


source /opt/ros/${ROS_DISTRO}/setup.bash

# Install source code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
wstool merge -t src src/${REPOSITORY_NAME}/${ROS_DISTRO}.rosinstall
wstool merge -t src src/${REPOSITORY_NAME}/travis.rosinstall
wstool update -t src
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -p1 -j1 --no-status
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 motion_planning
catkin_test_results --verbose build || catkin_test_results --all build
