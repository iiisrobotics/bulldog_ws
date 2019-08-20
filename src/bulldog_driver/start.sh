#!/bin/bash

sleep 3

if [  -f /opt/ros/indigo/setup.bash ];then
     source /opt/ros/indigo/setup.bash
fi


if [  -f /home/bulldog/bulldog_hardware_ws/devel/setup.bash ];then
     source /home/bulldog/bulldog_hardware_ws/devel/setup.bash
fi

export ROS_MASTER_URI=http://192.168.1.30:11311
export ROS_IP=192.168.1.30
echo "ROS environment is Ready"

roslaunch bulldog_driver bulldog_base.launch
