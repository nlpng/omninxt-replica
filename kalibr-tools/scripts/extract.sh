#!/bin/bash

# Use the first argument as the tag for the Docker image
# e.g. melodic, noetic
if [ -z "$1" ]; then
    echo "Usage: $0 <tag>"
    exit 1
fi
TAG=$1

if [ "$TAG" == "melodic" ]; then
    docker run -it --rm --privileged \
        -v $(pwd)/data:/app/data \
        --network host \
        --name=extractor quadcam-calib:$TAG bash -c 'python extract.py'
else
    docker run -it --rm --privileged \
        -v $(pwd)/data:/app/data \
        --network host \
        --name=extractor quadcam-calib:$TAG bash -c 'python3 extract.py'
fi
