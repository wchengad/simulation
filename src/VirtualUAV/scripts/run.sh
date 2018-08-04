#!/bin/bash

JOB_PARAMS=${1:-'--idx 0 --ishape 0 --stride 50 --seq_name h36m_S5_Waiting&1'} # defaults to [0, 0, 50]

# SET PATHS HERE
FFMPEG_PATH=/home/chengwei/Libs/ffmpeg/ffmpeg_build_sequoia_h264
X264_PATH=/home/chengwei/Libs/ffmpeg/x264_build
PYTHON2_PATH=/usr # PYTHON 2
BLENDER_PATH=/home/chengwei/Apps/blender/
#cd surreal/datageneration

# BUNLED PYTHON
BUNDLED_PYTHON=${BLENDER_PATH}/2.78/python
export PYTHONPATH=${BUNDLED_PYTHON}/lib/python3.5:${BUNDLED_PYTHON}/lib/python3.5/site-packages:/usr/local/lib/python3.5/dist-packages
export PYTHONPATH=${BUNDLED_PYTHON}:${PYTHONPATH}:/usr/local/lib/python/dist-packages

# FFMPEG
export LD_LIBRARY_PATH=${FFMPEG_PATH}/lib:${X264_PATH}/lib:${LD_LIBRARY_PATH}
export PATH=${FFMPEG_PATH}/bin:${PATH}

### RUN PART 1  --- Uses python3 because of Blender
$BLENDER_PATH/blender -b -P main_part1.py -- ${JOB_PARAMS}

### RUN PART 2  --- Uses python2 because of OpenEXR
PYTHONPATH="" ${PYTHON2_PATH}/bin/python2.7 main_part2.py ${JOB_PARAMS}
