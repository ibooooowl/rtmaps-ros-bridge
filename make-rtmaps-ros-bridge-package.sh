#!/bin/bash

## This script create the RTMaps ROS Bridge packages
OS=$(lsb_release -si)
OS_VER=$(lsb_release -sr)
ARCH=$(uname -m)
ROS_VER=$(rosversion -d)
BUILD_CORE_FUNCTION=rtmaps_ros_bridge_core_function.u/build
BUILD_BRIDGE=rtmaps_ros_bridge.u/build
BUILD_RMP_MSGS=rmp_msgs/build
TARGET_BRIDGE=rtmaps_ros_bridge.u/build/rtmaps_ros_bridge.pck
TARGET_CORE_FUNCTION=rtmaps_ros_bridge_core_function.u/build/rtmaps_ros_bridge_core_function.pck
package_name=rtmaps-ros-bridge


if [[ -e $BUILD_CORE_FUNCTION ]]; then
  rm -rf $BUILD_CORE_FUNCTION
fi


if [[ -e $BUILD_BRIDGE ]]; then
  rm -rf $BUILD_BRIDGE
fi

if [[ -e $BUILD_RMP_MSGS ]]; then
  rm -rf $BUILD_RMP_MSGS
fi


if [[ -f $TARGET_BRIDGE ]]; then
  rm $TARGET_BRIDGE
fi

if [[ -f $TARGET_CORE_FUNCTION ]]; then
  rm $TARGET_CORE_FUNCTION
fi
 
export RTMAPS_SDKDIR=/opt/rtmaps

mkdir -p rmp_msgs/build
cd rmp_msgs/build
cmake ..
make
source devel/setup.bash
cd ../..

#compile the RTMaps ROS Bridge Core Function
mkdir -p $BUILD_CORE_FUNCTION
cd $BUILD_CORE_FUNCTION
cmake ..
make
cd ../..

mkdir -p $BUILD_BRIDGE
cd $BUILD_BRIDGE
cmake ..
make
cd ../..



if [[ -f $TARGET_BRIDGE ]]; then
  cp $TARGET_BRIDGE .
fi

if [[ -f $TARGET_CORE_FUNCTION ]]; then
  cp $TARGET_CORE_FUNCTION .
fi
 
