#!/bin/bash
roslaunch yolov8_ros yolo_v8.launch &
sleep 30
echo "bhand controller startingsuccess!"
roslaunch plan_and_control trajectory_replan.launch &
sleep 0.1

wait
exit 0
