#!/bin/bash
xhost + 

# Use the first argument as the tag for the Docker image
# e.g. melodic, noetic
if [ -z "$1" ]; then
    echo "Usage: $0 <tag>"
    exit 1
fi
TAG=$1

if [ "$TAG" == "melodic" ]; then
    docker run -it --rm --privileged \
        -e DISPLAY=$DISPLAY \
        -v $(pwd)/data/extracted_output:/data \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --network host \
        --name=calibrator quadcam-calib:$TAG bash -c 'python calibrate.py'
else
    docker run -it --rm --privileged \
        -e DISPLAY=$DISPLAY \
        -v $(pwd)/data/extracted_output:/data \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --network host \
        --name=calibrator quadcam-calib:$TAG bash -c 'python3 calibrate.py'
fi
