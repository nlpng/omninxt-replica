#!/usr/bin/env python
import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from cv_bridge import CvBridge


def calibPhotometric(img, photometric, is_rgb=True):
    if not is_rgb:
        ret = img.copy()
        if len(img.shape) == 3:
            ret = cv.cvtColor(ret, cv.COLOR_BGR2GRAY)
        ret = ret.astype(float)/photometric
    else:
        # Divide by photometric per channel
        ret = img.copy().astype(float)
        for i in range(img.shape[2]):
            ret[:, :, i] = ret[:, :, i]/photometric*0.7
    ret = np.clip(ret, 0, 255).astype(np.uint8)
    return ret


def calibPhotometricImgsIndividual(imgs, photometrics, is_rgb=True):
    photometric_calibed = []
    if photometrics is not None:
        # Convert to grayscale
        for i in range(len(imgs)):
            calibed = calibPhotometric(imgs[i], photometrics[i], is_rgb=is_rgb)
            photometric_calibed.append(calibed)
        return photometric_calibed
    else:
        return imgs


def findPhotometric(cul_img, verbose=False):
    # Average in polar coordinates
    w_2 = cul_img.shape[1] // 2
    h_2 = cul_img.shape[0] // 2
    w = cul_img.shape[1]
    h = cul_img.shape[0]
    cul_line = np.zeros((w_2, 1))       # Half the width of the image
    count_line = np.zeros((w_2, 1))     # Half the width of the image
    pixels = w_2 * 2
    for theta in range(0, 360, 1):
        for r in np.linspace(0, w_2, pixels):
            x = int(r * np.cos(theta))
            y = int(r * np.sin(theta))
            # Ignore the top 20 pixels to get rid of the propeller
            if x + w_2 < 0 or x + w_2 >= w or y + h_2 >= h or y + h_2 < 20 or int(r) >= w_2:
                continue
            cul_line[int(r)] += cul_img[y + h_2, x + w_2]
            count_line[int(r)] += 1
    cul_line = cul_line / count_line
    cul_line = (cul_line - np.min(cul_line)) / (np.max(cul_line) - np.min(cul_line))
    if verbose:
        plt.plot(cul_line, label="Photometric")
        plt.legend()
        plt.show()
    mask = np.zeros(cul_img.shape, dtype=np.float)
    for x in range(0, w):
        for y in range(0, h):
            r = int(np.sqrt((x - w_2) ** 2 + (y - h_2) ** 2))
            if r < w_2:
                mask[y, x] = cul_line[r]
    # Normalize the mask by min max to 0.0 - 1.0
    return mask


def calibPhotometricMask(
        bag_dir, out_dir, cam_topics, step=5, verbose=False, start_t=0.):
    # Read from preporcessed bag (expect msgs are image type)
    cul_imgs = []
    for cam in cam_topics:
        bag = rosbag.Bag(os.path.join(bag_dir, "{}.bag".format(cam)))
        num_imgs = bag.get_message_count(cam)
        print("{}: {}".format(cam, num_imgs))

        if (num_imgs == 0):
            raise ValueError("no {} imgs in bag".format(cam))

        bridge = CvBridge()
        cam_idx = cam_topics.index(cam)
        count = 0
        img_count = 1
        t0 = None
        for topic, msg, t in bag.read_messages():
            try:
                if t0 is None:
                    t0 = t
                if (t - t0).to_sec() < start_t:
                    continue

                if (topic == cam):

                    if count % step != 0:
                        count += 1
                        continue

                    if msg._type == "sensor_msgs/Image":
                        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    else:
                        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

                    if cul_imgs == []:
                        # Create float64 image from gray image
                        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                        cul_imgs = [np.zeros(gray.shape, dtype=np.float64) for _ in cam_topics]

                    # Add gaussain
                    cul_imgs[cam_idx] += cv.GaussianBlur(cv.cvtColor(img, cv.COLOR_BGR2GRAY), (5, 5), 0) / 255.0

                    # Show averaged image
                    if verbose:
                        avg = cul_imgs[cam_idx] / img_count
                        cv.imshow("Raw {}".format(cam), img)
                        cv.imshow("Avg {}".format(cam), avg)
                        cv.waitKey(1)

                    img_count += 1
                    if verbose:
                        c = cv.waitKey(1)
                        if c == ord('q'):
                            break
                    count += 1

            except KeyboardInterrupt:
                break

        mask = findPhotometric(cul_imgs[cam_idx] / img_count)
        cv.imwrite("{}/cam_{}_vig_mask.png".format(out_dir, cam_idx), mask * 255)

    mask = findPhotometric((cul_imgs[2] + cul_imgs[3]) / 2.0 / img_count)
    if verbose:
        cv.imshow("Avg mask", mask)
    cv.imwrite("{}/avg23_mask.png".format(out_dir), mask * 255)
