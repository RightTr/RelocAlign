#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""

if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit 1
fi
echo "ROS version is: "$ROS_VERSION

rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/

if [ -f package.xml ]; then
    rm package.xml
fi

if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    cp -f package_ROS1.xml package.xml
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    cp -f package_ROS2.xml package.xml
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2}
fi

popd > /dev/null
