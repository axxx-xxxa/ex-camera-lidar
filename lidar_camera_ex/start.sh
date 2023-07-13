#!/bin/bash
Cur_Dir=$(pwd)
echo $Cur_Dir

source $Cur_Dir/devel/setup.bash

cd $Cur_Dir/src/lidar_camera_calib/launch
roslaunch lidar_camera_calib.launch &
sleep 0.5

wait
exit 0
