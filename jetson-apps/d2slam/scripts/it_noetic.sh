#Run docker and open bash shell
docker run -it --rm --privileged \
        --network=host \
        --runtime nvidia \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix/:/tmp/.X11-unix \
        --name=interactive d2slam:noetic-r35.4.1 bash
