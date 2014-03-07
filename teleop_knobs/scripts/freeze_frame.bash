#!/usr/bin/env bash

FILENAME="point_cloud_frozen.bag"
TOPIC_SUB="/narrow_stereo_textured/points2"
TOPIC_PUB="/narrow_stereo_textured/points2_frozen"
BAG_PATH="/home/ruebenm/pr2_device_interface/teleop/teleop_knobs/bags/"

cd $BAG_PATH
rosbag record -l 1 -O $FILENAME $TOPIC_SUB
rosbag info $FILENAME
rosbag play -l --hz=2 $FILENAME $TOPIC_SUB:=$TOPIC_PUB
