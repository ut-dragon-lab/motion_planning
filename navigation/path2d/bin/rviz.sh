#!/bin/sh

if [ "$ROS_DISTRO" = "fuerte" ]
then
    echo "fuerte version rviz"
    rosrun rviz rviz -d `rospack find $1`/config/$1_for_fuerte.rviz
#path2d_for_fuerte.rviz

elif [ "$ROS_DISTRO" = "groovy" ]
then
    echo "groov version"
    rosrun rviz rviz -d `rospack find $1`/config/$1_for_groovy.rviz

else
    echo "wrong ros revision"

fi
