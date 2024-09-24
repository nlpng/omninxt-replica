#Run docker and open bash shell
docker run -it --rm --privileged \
        --network=host \
        --runtime nvidia \
        --gpus all \
        -v /dev:/dev \
        --name=omni-depth d2slam:noetic-r35.4.1 bash -c 'roslaunch quadcam_depth_est depth-node.launch'
