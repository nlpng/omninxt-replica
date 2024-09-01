#Run docker and open bash shell
docker run -it --rm --privileged \
        -v /dev:/dev \
        --device-cgroup-rule='c 189:* rmw' \
        --network host \
        --name=oak_ffc_4p comm:noetic bash -c 'roslaunch oak_ffc_4p quadcam.launch'
