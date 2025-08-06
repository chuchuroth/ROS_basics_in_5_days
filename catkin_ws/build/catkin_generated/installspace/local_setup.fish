#!/usr/bin/env fish
# generated from catkin/cmake/template/local_setup.fish.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time

if test -z $_CATKIN_SETUP_DIR
    set _CATKIN_SETUP_DIR /mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/install
end

set CATKIN_SETUP_UTIL_ARGS "--extend --local"
source "$_CATKIN_SETUP_DIR/setup.fish"

set -e CATKIN_SETUP_UTIL_ARGS
