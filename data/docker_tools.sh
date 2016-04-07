#!/bin/bash
# we cannot use set -e otherwise we would stop if our program has a non
# zero return value
#set -e

# this will return the current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
cd ${SCRIPT_DIR}

# this is some shell magic such that we can the shell output and
# the output in a variable
exec 5>&1
# pipefail will return the non zero return value of any command in the pipe
res=$(set -o pipefail; ./.docker_tools.py $* | tee >(cat - >&5))

# if docker tools succeeded we run the output as a shell script
if [[ "$?" == "1" ]];then
    echo "running command"
    echo -e "${res}"
    eval "${res}"
fi
