#!/usr/bin/env python
import os

import utils.configurator as conf

# Generate shell scripts for calibration
for i, topic in enumerate(conf.rosbag_name):
    if i == 0:
        continue

    if i <= len(conf.camera_topics):
        # mono calibration
        cmd_mono = """#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
rosrun kalibr tartan_calibrate --bag /data/{}.bag --target /app/configs/april_6x6.yaml --topics {} --models omni-radtan --save_dir /data/{} --dont-show-report  """.format(
            topic, topic, topic
        )
        with open("/app/scripts/{}.sh".format(topic), "w+") as f:
            f.write(cmd_mono)
        os.system("chmod +x /app/scripts/{}.sh".format(topic))

        # imu calibration
        cmd_imu = """#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
rosrun kalibr kalibr_calibrate_imu_camera --bag /data/{}.bag --target /app/configs/april_6x6.yaml --cams /data/{}/log1-camchain.yaml --imu /app/configs/imu.yaml --dont-show-report  """.format(
            topic, topic
        )
        with open("/app/scripts/{}_imu.sh".format(topic), "w") as f:
            f.write(cmd_imu)
        os.system("chmod +x /app/scripts/{}_imu.sh".format(topic))

    else:
        # stereo calibration
        topic_former = topic.split("-")[0]
        topic_latter = topic.split("-")[1]
        cmd = """#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
rosrun kalibr tartan_calibrate --bag /data/{}.bag --target /app/configs/april_6x6.yaml --topics {} {} \
    --models omni-radtan  omni-radtan --save_dir /data/{} --dont-show-report  """.format(
            topic, topic_former, topic_latter, topic
        )
        with open("/app/scripts/{}.sh".format(topic), "w+") as f:
            f.write(cmd)
        os.system("chmod +x /app/scripts/{}.sh".format(topic))
