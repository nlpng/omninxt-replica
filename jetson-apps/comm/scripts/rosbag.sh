docker run -it -d --rm --privileged \
    -v /home/jetson/sandbox/ros_dev/bags:/tmp\
    --network host \
    --name=rosbag comm:noetic bash -c 'rosbag record /oak_ffc_4p/assemble_image/compressed /mavros/imu/data_raw -O /tmp/quadcam_calib.bag'
