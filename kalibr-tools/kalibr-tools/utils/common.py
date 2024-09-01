#!/usr/bin/env python
import os

import rosbag


def split_image(img, num_subimages=4):
    # Split image vertically
    _, w = img.shape[:2]
    sub_w = w // num_subimages
    sub_imgs = []
    for i in range(num_subimages):
        sub_imgs.append(img[:, i * sub_w:(i + 1) * sub_w])
    return sub_imgs


def check_rosbag(image_topic, imu_topic, bag_path):
    if not os.path.exists(bag_path):
        print("Bag file:{} does not exist".format(bag_path))
        return False

    bag = rosbag.Bag(bag_path)
    if bag.get_message_count(image_topic) <= 0:
        print(
            "No message in topic: {} check configuration".format(image_topic)
        )
        bag.close()
        return False

    if bag.get_message_count(imu_topic) <= 0:
        print(
            "No message in topic: {} check configuration".format(imu_topic)
        )
        bag.close()
        return False
    bag.close()
    return True
