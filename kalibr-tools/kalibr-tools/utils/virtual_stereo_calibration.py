#!/usr/bin/env python
from __future__ import print_function

import os
import shutil

import cv2 as cv
import numpy as np
import rosbag
import tqdm
from cv_bridge import CvBridge

import utils.config_loader as config_loader
import utils.photometric_calibrator as photometricCalibration
from utils.common import split_image


# set photometric mask before run this tool
def calib_photometric(img, photometric, is_rgb=True):
    if not is_rgb:
        ret = img.copy()
        if len(img.shape) == 3:
            ret = cv.cvtColor(ret, cv.COLOR_BGR2GRAY)
        ret = ret.astype(float) / photometric
    else:
        # Divide by photometric per channel
        ret = img.copy().astype(float)
        for i in range(img.shape[2]):
            ret[:, :, i] = ret[:, :, i] / photometric * 0.7
    ret = np.clip(ret, 0, 255).astype(np.uint8)
    return ret


def IndividualPhotometricCalibration(imgs, photometric_dir_path, is_rgb=True):
    photometric_calibed = []
    photometric_imgs = []
    if os.path.exists(photometric_dir_path):
        for i in range(len(imgs)):
            photo_metric_cali_img = (
                cv.imread(
                    photometric_dir_path + "/cam_" + str(i) + "_vig_mask.png",
                    cv.IMREAD_GRAYSCALE,
                )
                / 255.0
            )
            photometric_imgs.append(photo_metric_cali_img)
        for i in range(len(imgs)):
            calibed = calib_photometric(imgs[i], photometric_imgs[i], is_rgb=is_rgb)
            photometric_calibed.append(calibed)
    else:
        photometric_calibed = imgs
    return photometric_calibed


def kalibrCalibratePinhole(
    topic_a, topic_b, bagfile, output_calib_name, init_focal_length=400, verbose=False
):
    import subprocess

    if verbose:
        print("topic_a", topic_a)
        print("topic_b", topic_b)
        print("bagfile", bagfile)
        print("output_calib_name", output_calib_name)
        print("verbose", verbose)
        print("init_focal_length", init_focal_length)

    bagpath = os.path.dirname(bagfile)
    if bagpath == "":
        bagpath = os.getcwd()
        print("bag path is set to ", bagpath)

    # prepare script
    cmd = """#!/bin/bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
export KALIBR_FOCAL_LENGTH_INIT_VALUE={}
rosrun kalibr kalibr_calibrate_cameras --bag {} --target /app/configs/april_6x6.yaml --models pinhole-radtan pinhole-radtan --approx-sync 0.01 --topics {} {}""".format(
        init_focal_length, bagfile, topic_a, topic_b
    )
    if not verbose:
        cmd += " --dont-show-report"
    cmd += """<<EOF
{}
{}
EOF""".format(
        init_focal_length, init_focal_length
    )

    with open("{}/{}.sh".format(bagpath, output_calib_name), "w") as f:
        f.write(cmd)
    os.system("chmod +x {}/{}.sh".format(bagpath, output_calib_name))

    try:
        process = subprocess.Popen(
            ". {}/{}.sh".format(bagpath, output_calib_name), shell=True
        )

        # Capture the output and error streams
        stdout, stderr = process.communicate()

        if process.returncode == 0:
            print("Script output:\n{}".format(stdout))
            print("Script error (if any):\n{}".format(stderr))
        else:
            raise subprocess.CalledProcessError(
                process.returncode, cmd=output_calib_name, output=stderr
            )

    except subprocess.CalledProcessError as e:
        print("Error occurred while running the script: {}".format(e))

    except Exception as e:
        print("An unexpected error occurred: {}".format(e))

    finally:
        # Check if the process is still running
        if process.poll() is None:
            # Terminate the process
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        print("calibrate virtual stereo {} done".format(output_calib_name))

    # rename calibration file
    res_camchain = "{}/{}-camchain.yaml".format(bagpath, output_calib_name)
    if os.path.isfile(res_camchain):
        os.rename(
            res_camchain,
            "{}/{}.yaml".format(bagpath, output_calib_name)
        )


def calibration_task(
    calib_gen, stereo_calib_gens, input_bag_path, height, width, verbose
):
    retries = 0
    max_retries = 2
    output_calib_name = "stereo_calib_{}_{}_{}_{}".format(
        calib_gen.cam_idx_a, calib_gen.cam_idx_b, height, width
    )
    calib_bag_path = os.path.join(
        os.path.dirname(input_bag_path), output_calib_name + ".bag"
    )
    report_pdf_path = os.path.join(
        os.path.dirname(calib_bag_path), "{}-report-cam.pdf".format(output_calib_name)
    )
    while retries < max_retries:
        try:
            if not os.path.exists(report_pdf_path):
                topic_l = "/cam_{}_{}/compressed".format(
                    calib_gen.cam_idx_a, calib_gen.idx_l
                )
                topic_r = "/cam_{}_{}/compressed".format(
                    calib_gen.cam_idx_b, calib_gen.idx_r
                )
                shutil.copy2(input_bag_path, calib_bag_path)
                kalibrCalibratePinhole(
                    topic_l,
                    topic_r,
                    calib_bag_path,
                    output_calib_name,
                    verbose=verbose,
                    init_focal_length=stereo_calib_gens[0].undist_l.focal_gen,
                )
                os.remove(calib_bag_path)

            print("{}: OK".format(report_pdf_path))
            break
        except KeyboardInterrupt:
            os.remove(calib_bag_path)
            print("Ctrl+C detected. Exiting.")
            break
        except Exception as e:
            retries += 1
            if retries < max_retries:
                print("{} Retrying... ({}/{})".format(e, retries, max_retries))
            else:
                os.remove(calib_bag_path)
                print("Max retries reached. Exiting.")
                break


def calibrate_virtual_stereo(
    input_bag,
    fov,
    width,
    height,
    fisheye_config,
    output_path,
    step,
    photometric_calibration=None,
    verbose=False,
):
    # Load fisheye parameter
    if not os.path.exists(fisheye_config):
        print("[INPUT ERROR]{} calibration parameters not exist".format(fisheye_config))
        return
    else:
        stereo_gens, _ = config_loader.LoadFisheyeParameter(
            fisheye_config, fov=fov, width=width, height=height
        )

    # prepare output directory
    if output_path != "":
        if not os.path.exists(output_path):
            os.makedirs(output_path)
            print("{} created".format(output_path))
        output_bag_name = os.path.join(
            output_path,
            "stereo_calibration_step_{}_width_{}_height_{}.bag".format(
                step, width, height
            ),
        )

    # Read photometric
    photometrics = []
    if photometric_calibration:
        print(
            "Loading photometric calibration images from {}".format(
                photometric_calibration
            )
        )
        for i in range(4):
            vig_png = os.path.join(
                photometric_calibration, "cam_{}_vig_mask.png".format(i)
            )
            if not os.path.exists(vig_png):
                print("{} does not exist".format(vig_png))
                exit(1)
            vig_cal_img = cv.imread(vig_png, cv.IMREAD_GRAYSCALE) / 255.0
            photometrics.append(vig_cal_img)
    else:
        photometrics = None

    # If bag already exists, no need to generate again
    if not os.path.exists(output_bag_name):
        # Read from bag
        output_bag = rosbag.Bag(output_bag_name, mode="w")
        bag = rosbag.Bag(input_bag)
        num_imgs = bag.get_message_count(
            "/oak_ffc_4p/assemble_image/compressed"
        ) + bag.get_message_count("/oak_ffc_4p/assemble_image")
        print("Total number of images:", num_imgs)
        bridge = CvBridge()
        pbar = tqdm.tqdm(total=num_imgs // step, colour="green")
        count = 0
        for topic, msg, t in bag.read_messages():
            try:
                if (
                    topic == "/oak_ffc_4p/assemble_image/compressed"
                    or topic == "/oak_ffc_4p/assemble_image"
                ):
                    if count % step != 0:
                        count += 1
                        continue

                    if msg._type == "sensor_msgs/Image":
                        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    else:
                        img = bridge.compressed_imgmsg_to_cv2(
                            msg, desired_encoding="passthrough"
                        )

                    imgs = split_image(img)
                    calibed = photometricCalibration.calibPhotometricImgsIndividual(
                        imgs, photometrics, is_rgb=False
                    )

                    if verbose:
                        img_show = cv.hconcat(
                            [calibed[0], calibed[1], calibed[2], calibed[3]]
                        )
                        img_show_s = cv.resize(img_show, (1280, 240))
                        cv.imshow(
                            "calibrated img",
                            img_show_s,
                        )

                    for gen in stereo_gens:
                        cam_idx_a = gen.cam_idx_a
                        cam_idx_b = gen.cam_idx_b
                        idx_vcam_a = gen.idx_l
                        idx_vcam_b = gen.idx_r
                        img_l, img_r = gen.genStereo(
                            calibed[cam_idx_a], calibed[cam_idx_b]
                        )

                        if verbose:
                            img_show = cv.hconcat([img_l, img_r])
                            cv.imshow(
                                "stereo {}_{} <-> {}_{}".format(
                                    cam_idx_a, idx_vcam_a, cam_idx_b, idx_vcam_b
                                ),
                                img_show,
                            )
                        topic_l, topic_r = (
                            "/cam_{}_{}/compressed".format(cam_idx_a, idx_vcam_a),
                            "/cam_{}_{}/compressed".format(cam_idx_b, idx_vcam_b),
                        )
                        comp_img_l, comp_img_r = bridge.cv2_to_compressed_imgmsg(
                            img_l
                        ), bridge.cv2_to_compressed_imgmsg(img_r)
                        comp_img_l.header = comp_img_r.header = msg.header
                        output_bag.write(topic_l, comp_img_l, t)
                        output_bag.write(topic_r, comp_img_r, t)
                    pbar.update(1)
                    if verbose:
                        c = cv.waitKey(1)
                        if c == ord("q"):
                            break
                    count += 1
            except KeyboardInterrupt:
                break
        output_bag.close()

    else:
        print("Using the previours bag: {}".format(output_bag_name))

    for gen in stereo_gens:
        calibration_task(gen, stereo_gens, output_bag_name, height, width, verbose)

    # prefer do it with multiprocessing pool with 4 processes
    # try:
    #     pool = multiprocessing.Pool(
    #         processes=4,
    #         initializer=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))
    #     for gen in stereo_gens:
    #         # Apply the calibration_task function to each generator
    #         pool.apply_async(
    #             calibration_task,
    #             args=(
    #                 gen,
    #                 stereo_gens,
    #                 output_bag_name,
    #                 height,
    #                 width,
    #                 verbose,
    #             )
    #         )
    #     pool.close()
    #     pool.join()
    # except KeyboardInterrupt:
    #     # TODO: Ctrl+C still not working
    #     print("Ctrl+C detected. Stopping all tasks...")
    #     pool.terminate()
    #     pool.join()
    #     sys.exit(1)
