#!/bin/bash

#-------------------------------------------------------------------------------
# Verify that this script is being called from the top level of a workspace.
echo "Verifying that the script is being called from the top level of a catkin workspace..."
catkin_ws_error_msg="Note: This script should be called from the top level of a catkin workspace."
# Check for src
if [ ! -d src ]; then
  echo -e $catkin_ws_error_msg
  echo -e "Did not find src subdirectory"
  exit 1
fi
# Check for CMakeLists.txt
if [ ! -f src/CMakeLists.txt ]; then
  echo -e $catkin_ws_error_msg
  echo -e "Did not find src/CMakeLists.txt"
  exit 1
fi
# Check that the CMakeLists.txt is a catkin toplevel cmake lists
res=`grep CATKIN_TOPLEVEL src/CMakeLists.txt`
if [ "$res" == "" ]; then
  echo -e $catkin_ws_error_msg
  echo "src/CMakeLists.txt does not seem to be a top-level catkin CMakeLists.txt file"
  exit 1
fi
echo "[success]"
#-------------------------------------------------------------------------------

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# copied from get.docker.com
command_exists() {
	command -v "$@" > /dev/null 2>&1
}

user_confirm() {
    NOT_FINISHED=true
    while ${NOT_FINISHED} ;do
        echo -e "$1 [y/n] default($2) "
        read USER_INPUT;
        if [[ "y" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="y";
            NOT_FINISHED=false;
        elif [[ "n" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="n";
            NOT_FINISHED=false;
        elif [[ "" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="$2";
            NOT_FINISHED=false;
        else
            echo -e "# only y, n, and nothing, are possible choices."
            echo -e "# default is $2"
        fi
    done
}

do_install() {
###############################################################
# we make sure that we have docker installed
###############################################################
if ! command_exists docker; then
    echo -e "###############################################################"
    echo -e "# !!!! IMPORTANT !!!! "
    echo -e "# Docker is not yet installed, we are now installing it."
    echo -e "# This procedure might ask for sudo rights."
    echo -e "# To finish this installation, you have to restart your computer."
    echo -e "# We wait for 10 sec until we start the installation process."
    echo -e "###############################################################"
    sleep 10;
    wget -qO- https://get.docker.com | sh
    sudo usermod -a -G docker $USER
    echo -e "###############################################################"
    echo -e "# !!!! IMPORTANT !!!! "
    echo -e "# We will restart your machine in 10 sec."
    echo -e "# Please RUN this script AGAIN afterwards to finish the initialization."
    echo -e "###############################################################"
    sleep 10;
    sudo shutdown -r now
    exit 0;
fi

if ! command_exists catkin_init_workspace; then
    echo -e "###############################################################"
    echo -e "# ros groovy or indigo is not sourced"
    echo -e "###############################################################"
    if [[ -d "/opt/ros/indigo" ]];then
        echo -e "###############################################################"
        echo -e "# found /opt/ros/indigo/setup.bash"
        echo -e "###############################################################"
        source /opt/ros/indigo/setup.bash

    elif [[ -d "/opt/ros/indigo" ]];then
        echo -e "###############################################################"
        echo -e "# found /opt/ros/groovy/setup.bash"
        echo -e "###############################################################"
        source /opt/ros/groovy/setup.bash
    else
        echo -e "###############################################################"
        echo -e "# ros groove or indigo is require, please install the dependency or source the setup.bash in ,your ros distribution prior to calling this script."
        echo -e "# http://wiki.ros.org/indigo/Installation/Ubuntu"
        echo -e "###############################################################"
        exit 1;
    fi
fi

echo -e "###############################################################"
echo -e "# Set your docker credentials if you haven't already. This only "
echo -e "# Needs to be done once, the first time through:"
echo -e "#   Username: uw"
echo -e "#   Password: private communication"
echo -e "#   Email: your choice :)"
user_confirm "# Update credentials" "n"
if [[ "y" == "${USER_CONFIRM_RESULT}" ]];then
    docker login hub.lularobotics.com 
fi

echo -e "###############################################################"
echo -e "# We will update binary software package with the latest version"
echo -e "# This could take several minutes the first time through. After"
echo -e "# that the update should be fast."
user_confirm "# Continue" "y"
if [[ "n" == "${USER_CONFIRM_RESULT}" ]];then
    echo -e "stopped since user did want to stop";
    exit 1;
fi

bash ${SCRIPT_DIR}/data/docker_tools.sh --load-image

echo -e "###############################################################"
echo -e "# We will add the lula packages to a 'lula' subdirectory of this"
echo -e "# workspace if they are not already there. If they are already "
echo -e "# there, we will update them."
user_confirm "# Continue" "y"
if [[ "n" == "${USER_CONFIRM_RESULT}" ]];then
    echo -e "stopped since user did not confirm the package installation";
    exit 1;
fi

################################################################################
# Set up the workspace. By convention, this script should be called from the 
# root of the desired workspace. That's verified up front. Add our packages
# to a lula subdirectory of the src folder of the workspace. If we do not 
# find a baxter_common, ask whether we should install it ourselves. If we 
# do find one, ask whether we should update it.
################################################################################
${SCRIPT_DIR}/update_packages.sh
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo;

echo -e "#####################################################################"
echo -e "# Your workspace and Lula docker image are now setup:"
echo -e "# See the readme.txt to run the demos and check the installation."
echo -e "#####################################################################"
}

# copied from get.docker.com
# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
