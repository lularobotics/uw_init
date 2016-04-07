#!/bin/bash
set -e

# copied from get.docker.com
command_exists() {
	command -v "$@" > /dev/null 2>&1
}

if [[ $# != 1 ]];then
    echo -e "###############################################################"
    echo -e "# we require exactly one parameters the path to lularobotics_uw"
    echo -e "###############################################################"
    exit 1
fi

DIR_LULAROBOTICS_UW="$1"

if [[ ! -d ${DIR_LULAROBOTICS_UW} ]];then
    echo -e "###############################################################"
    echo -e "# the path to lularobotics_nw ${DIR_LULAROBOTICS_UW} is not a directory"
    echo -e "###############################################################"
    exit 1
fi

cd ${DIR_LULAROBOTICS_UW};

if [[ ! -e ${DIR_LULAROBOTICS_UW}/devel/setup.bash ]];then
    echo -e "###############################################################"
    echo -e "# the workspace ${DIR_LULAROBOTICS_UW} seems not to be a compiled"
    echo -e "# catkin workspace. Please call catkin_make in ${DIR_LULAROBOTICS_UW}"
    echo -e "###############################################################"
    exit 1
fi
source ${DIR_LULAROBOTICS_UW}/devel/setup.bash

if ! command_exists roscore; then
    echo -e "###############################################################"
    echo -e "# ros seems to be not installed properly or"
    echo -e "# the setup.bash was not sourced properly."
    echo -e "###############################################################"
    exit 1
fi

echo -e "###############################################################"
echo -e "# starting up the roscore"
echo -e "###############################################################"
roscore > /dev/null 2>&1&
sleep 1;
echo -e "###############################################################"
echo -e "# MORE INSTRUCTIONS HERE ..."
echo -e "###############################################################"

echo -e "###############################################################"
echo -e "# we are planning in the background please wait"
echo -e "# the robot should move soon :)"
echo -e "###############################################################"
read USER_INPUT;

echo -e "###############################################################"
echo -e "# CALL STOPPING PROCEDURES HERE "
echo -e "###############################################################"
