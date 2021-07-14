#!/bin/bash

# check that the number of received arguments are correct
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash create_robocup_pkg.sh name_of_my_awesome_pkg"
    exit
fi

# assign first received argument to variable PKG_NAME
PKG_NAME=${1}
AUTHOR_NAME="multi-robot-matlab-gspnr"
AUTHOR_EMAIL="multi-robot-matlab-gspnr"
MAINTAINER_NAME="multi-robot-matlab-gspnr"
MAINTAINER_EMAIL="multi-robot-matlab-gspnr"

# setup file content
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 3.0.2)\nproject(${PKG_NAME})\n\nfind_package(catkin REQUIRED COMPONENTS\n actionlib\n actionlib_msgs\n actionlib_tutorials\n rospy\n std_msgs\n roslaunch\n)\n\ncatkin_python_setup()\n\ncatkin_package(\n    CATKIN_DEPENDS actionlib actionlib_msgs actionlib_tutorials rospy std_msgs\n)\n\nroslaunch_add_file_check(launch/matlab_interface_servers.launch)"
PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>\n<package>\n  <name>${PKG_NAME}</name>\n  <version>1.0.0</version>\n  <description>\n    TODO!\n  </description>\n\n <maintainer email=\"antonio@todo.todo\">antonio</maintainer>\n\n <license>GPLv3</license>\n\n  \n  <buildtool_depend>catkin</buildtool_depend>\n\n  <build_depend>actionlib</build_depend>\n<build_depend>actionlib_msgs</build_depend>\n<build_depend>actionlib_tutorials</build_depend>\n<build_depend>rospy</build_depend>\n<build_depend>std_msgs</build_depend>\n\n<run_depend>actionlib</run_depend>\n<run_depend>actionlib_msgs</run_depend>\n<run_depend>actionlib_tutorials</run_depend>\n<run_depend>rospy</run_depend>\n<run_depend>std_msgs</run_depend>\n\n</package>"
SETUP_PY_CONTENT="#!/usr/bin/env python\n\nfrom distutils.core import setup\nfrom catkin_pkg.python_setup import generate_distutils_setup\n\n# for your packages to be recognized by python\nd = generate_distutils_setup(\n  packages=['${PKG_NAME}'],\n  package_dir={'': 'src'}\n)\n\nsetup(**d)"

TEMP="$(cut -d':' -f1 <<<"$CMAKE_PREFIX_PATH")"
CATKIN_WS=${TEMP: : -6}
CATKIN_WS=$CATKIN_WS/src

mkdir $CATKIN_WS/${PKG_NAME}
# cd ${PKG_NAME}
mkdir $CATKIN_WS/${PKG_NAME}/src
mkdir $CATKIN_WS/${PKG_NAME}/launch

# create file structure
touch $CATKIN_WS/${PKG_NAME}/CMakeLists.txt
touch $CATKIN_WS/${PKG_NAME}/package.xml
touch $CATKIN_WS/${PKG_NAME}/setup.py

# fill files with information from parameter section
echo -e $CMAKELISTS_CONTENT  > $CATKIN_WS/${PKG_NAME}/CMakeLists.txt
echo -e $PACKAGE_XML_CONTENT > $CATKIN_WS/${PKG_NAME}/package.xml
echo -e $SETUP_PY_CONTENT > $CATKIN_WS/${PKG_NAME}/setup.py


# catkin build --this
# source ~/.bashrc
