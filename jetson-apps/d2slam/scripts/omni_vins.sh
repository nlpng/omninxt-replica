xhost +

#Run docker and open bash shell
docker run -it --rm --privileged \
        --network=host \
        --runtime nvidia \
        --gpus all \
        --env="DISPLAY=:0" \
	--env="QT_X11_NO_MITSHM=1" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        --name=omni-vins d2slam:noetic-r35.4.1 bash -c 'roslaunch d2vins vins-node.launch'
