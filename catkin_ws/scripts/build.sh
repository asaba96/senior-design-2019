#!/usr/bin/env bash

pushd $(dirname ${BASH_SOURCE}) > /dev/null
set -e
catkin build -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
popd > /dev/null
