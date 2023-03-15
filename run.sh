#!/bin/bash

ORB_SLAM3_DIR=${HOME}/ucl-cs/y4/fyp/ORB_SLAM3
PANGOLIN_DIR=${HOME}/ucl-cs/y4/fyp/Pangolin


PANGOLIN=${PANGOLIN_DIR}/build
ORB_DBOW2=${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib
ORB_G2O=${ORB_SLAM3_DIR}/Thirdparty/g2o/lib
ORB_SLAM=${ORB_SLAM3_DIR}/lib

export LD_LIBRARY_PATH=${PANGOLIN}:${ORB_DBOW2}:${ORB_G2O}:${ORB_SLAM}:${LD_LIBRARY_PATH}

exec "$@"