#!/bin/bash

## This script create the RTMaps ROS Bridge packages
OS=$(lsb_release -si)
OS_VER=$(lsb_release -sr)
ARCH=$(uname -m)
ROS_VER=$(rosversion -d)

package_name=rtmaps-ros-bridge

if [ "$1" == "" ]
then
	echo "Please, give a version number"
	echo "usage : $0 x.y.z"
	exit
else
	version="$1"
	echo "version : $version"
fi

temp_dir=./rtmaps-ros-bridge-temp
soft_dest=$temp_dir/opt/rtmaps

# Create the temp skeleton directory
mkdir -p $temp_dir
cp -lR rtmaps-ros-bridge/* $temp_dir/
mkdir -p $soft_dest
mkdir -p $soft_dest/packages
mkdir -p $soft_dest/doc/studio_reference/components
mkdir -p $soft_dest/doc/studio_reference/resources

cp -l ../packages/${OS,,}${OS_VER//./}_${ARCH}/rtmaps_ros_bridge.pck $soft_dest/packages/
cp -lR ../rtmaps_ros_bridge.u/doc/* $soft_dest/doc/

### Delete useless files
find $temp_dir/ -name ".svn" -type d -exec rm -rf {} \; 2>/dev/null

### Calculate installed packet size
size=$(du -sk $temp_dir | cut -f 1)

control_file=$temp_dir/DEBIAN/control
### Modify the "control" file with given version number and size
sed -i -e "s/__version__/$version/g" $control_file
sed -i -e "s/__size__/$size/g" $control_file

### Create the package
dpkg -b $temp_dir ${package_name}_${version}_${OS}_${OS_VER}_${ARCH}_ros_${ROS_VER}.deb

### Cleanup
rm -rf $temp_dir
