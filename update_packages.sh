#!/bin/bash

################################################################################
# Set up the workspace. By convention, this script should be called from the 
# root of the desired workspace. That's verified up front. Add our packages
# to a lula subdirectory of the src folder of the workspace. If we do not 
# find a baxter_common, ask whether we should install it ourselves. If we 
# do find one, ask whether we should update it.
################################################################################

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

prompt_line() {
  echo
  echo "------------------------------------------------------------------------"
}

found_only_one_rethink_package_error() {
  found_package=$1
  not_found_package=$2
  echo -e "WARNING -- Did not find '$not_found_package' package, but did find "
  echo -e "'$found_package' package. Please make sure both of these packages "
  echo -e "are installed in your active workspace before continuing. They can be "
  echo -e "installed in combination using:"
  echo -e "  git clone https://github.com/RethinkRobotics/baxter_common.git"
  echo -e
  echo -e "If you remove '$found_package', we can install this for you."
}

manually_install_rethink_packages_error() {
  echo -e "Please make sure both 'baxter_description' and 'rethink_ee_description'"
  echo -e "are installed in the workspace before continuing. They can be installed"
  echo -e "using:"
  echo -e "  git clone https://github.com/RethinkRobotics/baxter_common.git"
}

lula_dir=src/lula
if [[ ! -d $lula_dir ]]; then
  echo "Creating Lula package directories"
  mkdir -p $lula_dir
  pushd $lula_dir
  git clone https://github.com/lularobotics/lula_baxter.git
  popd  # $lula_dir

  echo "Searching for baxter_common..."

  # Check whether either or both of the baxter_description and 
  # rethink_ee_description packages are there.
  baxter_description_dir=$(find src -type d -name 'baxter_description')
  rethink_ee_description_dir=$(find src -type d -name 'rethink_ee_description')
  found_count=0
  if [ "$baxter_description_dir" != "" ]; then
    ((found_count++))
  fi
  if [ "$rethink_ee_description_dir" != "" ]; then
    ((found_count++))
  fi

  if [ "$found_count" == 2 ]; then
    # We found both packages and we can just use that if the user says it's ok.

    prompt_line
    echo "Found both Baxter description packages already installed in this workspace:"
    echo "  $baxter_description_dir"
    echo "  $rethink_ee_description_dir"
    user_confirm "# Use these packages?" "y"
    if [[ "n" == "${USER_CONFIRM_RESULT}" ]];then
      prompt_line
      manually_install_rethink_packages_error
      exit 1
    fi
  elif [ "$found_count" == 1 ]; then
    # Only found one of baxter_description or rethink_ee_description. This is
    # something that needs to be resolved manually.
    if [ "$baxter_description_dir" == "" ]; then
      prompt_line
      found_only_one_rethink_package_error \
        "rethink_ee_description" \
        "baxter_description" 
    fi
    if [ "$rethink_ee_description_dir" == "" ]; then
      prompt_line
      found_only_one_rethink_package_error \
        "baxter_description" \
        "rethink_ee_description"
    fi
    exit 1
  else
    # We didn't find either of the baxter description packages, so install
    # baxter_common manually. 
    prompt_line
    echo "Baxter model packages not found in this workspace."
    user_confirm "# Install these automatically now by cloning baxter_common?" "y"
    if [[ "y" == "${USER_CONFIRM_RESULT}" ]]; then
      pushd $lula_dir
      git clone https://github.com/RethinkRobotics/baxter_common.git
      popd  # $lula_dir
    else
      manually_install_rethink_packages_error
      exit 1
    fi
  fi
else
  echo "Updating Lula packages..."
  pushd $lula_dir

  # If there were more packages, we'd have one of these blocks per package.
  echo
  echo "[updating lula_baxter]"
  pushd lula_baxter
  git pull
  popd

  # If we've installed baxter_common then update them now. Otherwise, let
  # the user to handle those updates.
  if [ -d baxter_common ]; then
    echo
    echo "[updating baxter_common]"
    pushd baxter_common
    git pull
    popd
  fi

  popd  # $lula_dir
  echo
  echo "[done]"
fi
