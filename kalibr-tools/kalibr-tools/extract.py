#!/usr/bin/env python
import os

import utils.bag_extractor as bag_extractor
import utils.common as common_utils
import utils.configurator as conf
import utils.photometric_calibrator as photometric_calibrator

ret = common_utils.check_rosbag(
    conf.image_topic, conf.imu_topic, conf.rosbag_path)
if not ret:
    exit(1)

are_extracted = {}
for n in conf.rosbag_name[1:]:
    are_extracted[n] = os.path.exists(conf.output_path + "/" + n + ".bag")

if not all(list(are_extracted.values())):
    # Split bag into 9 parts for calibration maybe slow if bag is large
    bags = bag_extractor.extract_calibration_bag(
        conf.rosbag_path, conf.output_path, conf.image_topic
    )
    print("[Bag extractor] The following bags are created:\n")
    for bag_file in bags:
        print(bag_file)
else:
    print("[Bag extractor] Skip extration! bags are already created.")

# Splitted bags are generated at this point
# prepare photometric mask saving path
mask_path = os.path.join(conf.output_path, "quadcam_vig_mask")
if not os.path.exists(mask_path):
    os.makedirs(mask_path)
    print("{} created".format(mask_path))

photometric_calibrator.calibPhotometricMask(
    conf.output_path, mask_path, conf.camera_topics
)
