#!/usr/bin/env zsh

set -e

# multi-agent-ros
make format

# loop over submodules
for dir in ./submodules/*
do
    if [ -f $dir/Makefile ]; then
        cd $dir
        echo "Running format in $dir................"
        make format
        cd -
    fi
done