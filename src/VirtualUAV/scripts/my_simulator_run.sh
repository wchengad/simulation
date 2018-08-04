#!/bin/bash

#JOB_PARAMS=${1:-'--init_camera_pose 1.0,0.5,1.0,0.78'} # defaults to [0, 0, 50]

# SET PATHS HERE
FFMPEG_PATH=/home/chengwei/Libs/ffmpeg/ffmpeg_build_sequoia_h264
X264_PATH=/home/chengwei/Libs/ffmpeg/x264_build
PYTHON2_PATH=/usr # PYTHON 2
BLENDER_PATH=/home/chengwei/Apps/blender/
PROJECT_PATH=/home/chengwei/Projects/Simulator/
#cd surreal/datageneration

# BUNLED PYTHON
BUNDLED_PYTHON=${BLENDER_PATH}/2.78/python
export PYTHONPATH=${BUNDLED_PYTHON}/lib/python3.5:${BUNDLED_PYTHON}/lib/python3.5/site-packages:/usr/local/lib/python3.5/dist-packages
export PYTHONPATH=${BUNDLED_PYTHON}:${PYTHONPATH}:/usr/local/lib/python/dist-packages

# FFMPEG
export LD_LIBRARY_PATH=${FFMPEG_PATH}/lib:${X264_PATH}/lib:${LD_LIBRARY_PATH}
export PATH=${FFMPEG_PATH}/bin:${PATH}

echo $1

### RUN PART 1  --- Uses python3 because of Blender
$BLENDER_PATH/blender -P $PROJECT_PATH/src/VirtualUAV/scripts/my_simulator.py -- $1
