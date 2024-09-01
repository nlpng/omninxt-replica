# Run docker and open bash shell
docker run -it -d --rm --privileged \
        -v /dev:/dev \
        --network host \
        --name=mavros comm:noetic bash -c 'roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS0:921600" '
