#!/usr/bin/env bash

PROJDIR="$PWD"

set -x
set -e

cd "$PROJDIR/3rdParty/RSSceneViewer"
mkdir -p include lib
cd src
cp RSSceneViewer.h RSSceneViewer_global.h ../include
mkdir -p build && cd build
cmake ..
make && cp libRSSceneViewer.so ../../lib
cd ..
rm -Rf build

# should work for Ubuntu 20.04
#cd "$PROJDIR/src/livox_ros_driver2/"
#source /opt/ros/noetic/setup.sh
#./build.sh ROS1

cd "$PROJDIR"
catkin_make -DCATKIN_WHITELIST_PACKAGES="rshandheld_capture"



