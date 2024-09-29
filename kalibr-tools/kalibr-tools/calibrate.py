#!/usr/bin/env python
import os
import subprocess
import sys

import utils.configurator as conf
import utils.virtual_stereo_calibration as vsc
import yaml


def generate_d2vins_configuration(output_path):
    camera_a_yaml = open(output_path + "/CAM_A/log1-camchain.yaml", "r")
    camera_a_intrinsic = yaml.safe_load(camera_a_yaml)["cam0"]
    camera_a_yaml.close()
    camera_b_yaml = open(output_path + "/CAM_B/log1-camchain.yaml", "r")
    camera_b_intrinsic = yaml.safe_load(camera_b_yaml)["cam0"]
    camera_b_yaml.close()
    camera_c_yaml = open(output_path + "/CAM_C/log1-camchain.yaml", "r")
    camera_c_intrinsic = yaml.safe_load(camera_c_yaml)["cam0"]
    camera_c_yaml.close()
    camera_d_yaml = open(output_path + "/CAM_D/log1-camchain.yaml", "r")
    camera_d_intrinsic = yaml.safe_load(camera_d_yaml)["cam0"]
    camera_d_yaml.close()

    # generate d2vins configuration
    d2vins_config_yaml = open(os.path.join(output_path, "fisheye_cams.yaml"), "w")
    d2vins_config = {}
    d2vins_config["cam0"] = camera_a_intrinsic
    d2vins_config["cam1"] = camera_b_intrinsic
    d2vins_config["cam2"] = camera_c_intrinsic
    d2vins_config["cam3"] = camera_d_intrinsic
    cam_1_2_yaml = open(output_path + "/CAM_A-CAM_B/log1-camchain.yaml", "r")
    cam_1_2_extrinsic = yaml.safe_load(cam_1_2_yaml)["cam1"]["T_cn_cnm1"]
    cam_1_2_yaml.close()
    cam_2_3_yaml = open(output_path + "/CAM_B-CAM_C/log1-camchain.yaml", "r")
    cam_2_3_extrinsic = yaml.safe_load(cam_2_3_yaml)["cam1"]["T_cn_cnm1"]
    cam_2_3_yaml.close()
    cam_3_4_yaml = open(output_path + "/CAM_C-CAM_D/log1-camchain.yaml", "r")
    cam_3_4_extrinsic = yaml.safe_load(cam_3_4_yaml)["cam1"]["T_cn_cnm1"]
    cam_3_4_yaml.close()
    d2vins_config["cam1"]["T_cn_cnm1"] = cam_1_2_extrinsic
    d2vins_config["cam2"]["T_cn_cnm1"] = cam_2_3_extrinsic
    d2vins_config["cam3"]["T_cn_cnm1"] = cam_3_4_extrinsic

    # cam_imu
    cam_a_imu_yaml = open(os.path.join(output_path, "CAM_A-camchain-imucam.yaml"), "r")
    cam_a_imu_extrinsic = yaml.safe_load(cam_a_imu_yaml)["cam0"]["T_cam_imu"]
    cam_a_imu_yaml.close()
    cam_b_imu_yaml = open(os.path.join(output_path, "CAM_B-camchain-imucam.yaml"), "r")
    cam_b_imu_extrinsic = yaml.safe_load(cam_b_imu_yaml)["cam0"]["T_cam_imu"]
    cam_b_imu_yaml.close()
    cam_c_imu_yaml = open(os.path.join(output_path, "CAM_C-camchain-imucam.yaml"), "r")
    cam_c_imu_extrinsic = yaml.safe_load(cam_c_imu_yaml)["cam0"]["T_cam_imu"]
    cam_c_imu_yaml.close()
    cam_d_imu_yaml = open(os.path.join(output_path, "CAM_D-camchain-imucam.yaml"), "r")
    cam_d_imu_extrinsic = yaml.safe_load(cam_d_imu_yaml)["cam0"]["T_cam_imu"]
    cam_d_imu_yaml.close()
    d2vins_config["cam0"]["T_cam_imu"] = cam_a_imu_extrinsic
    d2vins_config["cam1"]["T_cam_imu"] = cam_b_imu_extrinsic
    d2vins_config["cam2"]["T_cam_imu"] = cam_c_imu_extrinsic
    d2vins_config["cam3"]["T_cam_imu"] = cam_d_imu_extrinsic
    yaml.safe_dump(d2vins_config, d2vins_config_yaml, default_flow_style=False)
    print(
        "d2vins_config_yaml write into {}, exist: {}".format(
            os.path.join(output_path, "fisheye_cams.yaml"),
            os.path.exists(os.path.join(output_path, "fisheye_cams.yaml")),
        )
    )
    d2vins_config_yaml.close()


def check_calibration(output_path, calibration_type="mono"):
    num_cam = len(conf.camera_topics)
    if calibration_type == "mono":
        bag_names = conf.rosbag_name[1 : num_cam + 1]
    elif calibration_type == "stereo":
        bag_names = conf.rosbag_name[num_cam + 1 :]
    calibration_status = [False for _ in range(num_cam)]
    iter = 0
    failed = False
    for bag_name in bag_names:
        bag_path = output_path + "/" + bag_name + "/log1-camchain.yaml"
        if os.path.exists(bag_path):
            calibration_status[iter] = True
        iter += 1
    for i in range(4):
        if not calibration_status[i]:
            print(
                "calibration failed for bag {} \
                  record new bag and extract under the same name".format(
                    bag_names[i]
                )
            )
            failed = True

    if not failed:
        print("calibration success for all bags")

    return not failed


def check_imu_calibration(output_path):
    num_cam = len(conf.camera_topics)
    topics = conf.rosbag_name[1 : num_cam + 1]
    for topic in topics:
        bag_path = output_path + "/" + topic + "-camchain-imucam.yaml"
        if not os.path.exists(bag_path):
            print("imu calibration failed for {}".format(topic))
            return False
    print("imu calibration success for all bags")
    return True


if not os.listdir("/data"):
    print("bags not extracted, run ./script/extract.sh first")
    exit(1)

for i, topic in enumerate(conf.rosbag_name):
    if i == 0:
        continue

    if i <= len(conf.camera_topics):
        # attemp to skip re-calibration if calibration is completed
        done_mono = os.path.isfile("/data/{}/log1-camchain.yaml".format(topic))
        done_imu = os.path.isfile("/data/{}-camchain-imucam.yaml".format(topic))
        if os.path.exists("/data/{}".format(topic)) and done_mono and done_imu:
            print("mono {} seems calibrated, skip.".format(topic))
            continue

        # mono calibration
        try:
            # NOTE:
            # `subprocess.run` not available in Py2, use `subprocess.Popen`
            # result = subprocess.run(
            #     ['. /app/scripts/{}.sh'.format(topic)],
            #     check=True,
            #     shell=True
            # )
            process = subprocess.Popen(". /app/scripts/{}.sh".format(topic), shell=True)

            # Capture the output and error streams
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                print("Script output:\n{}".format(stdout))
                print("Script error (if any):\n{}".format(stderr))
            else:
                raise subprocess.CalledProcessError(
                    process.returncode, cmd=topic, output=stderr
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
                    # Wait for the process to terminate
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Forcefully kill the process if it doesn't terminate
                    process.kill()

            print("calibrate mono {} done".format(topic))

        # imu calibration
        try:
            process = subprocess.Popen(
                ". /app/scripts/{}_imu.sh".format(topic), shell=True
            )

            # Capture the output and error streams
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                print("Script output:\n{}".format(stdout))
                print("Script error (if any):\n{}".format(stderr))
            else:
                raise subprocess.CalledProcessError(
                    process.returncode, cmd=topic, output=stderr
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
                    # Wait for the process to terminate
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Forcefully kill the process if it doesn't terminate
                    process.kill()

            print("calibrate mono imu {} done".format(topic))

    else:
        # attemp to skip re-calibration stereo if calibration is completed
        done_stereo = os.path.isfile("/data/{}/log1-camchain.yaml".format(topic))
        if os.path.exists("/data/{}".format(topic)) and done_stereo:
            print("stereo {} seems calibrated, skip.".format(topic))
            continue

        # reset stereo config
        topic_former = topic.split("-")[0]
        topic_latter = topic.split("-")[1]
        print("topic_former {} topic_latter {}".format(topic_former, topic_latter))
        # remove former and latter
        if os.path.exists("/data/{}".format(topic)):
            os.system("rm -r /data/{}".format(topic))
        os.mkdir("/data/{}".format(topic))
        intrinsic_file = open("/data/{}/intrinsic.yaml".format(topic), "w")
        # generate better intrinsic
        former_cam_yaml = open("/data/{}/log1-camchain.yaml".format(topic_former), "r")
        former_cam_intrinsic = yaml.safe_load(former_cam_yaml)
        former_cam_yaml.close()
        former_cam_intrinsic = former_cam_intrinsic["cam0"]
        later_cam_yaml = open("/data/{}/log1-camchain.yaml".format(topic_latter), "r")
        later_cam_intrinsic = yaml.safe_load(later_cam_yaml)
        later_cam_yaml.close()
        later_cam_intrinsic = later_cam_intrinsic["cam0"]
        intrinsic_yaml = {}
        intrinsic_yaml["cam0"] = former_cam_intrinsic
        intrinsic_yaml["cam1"] = later_cam_intrinsic
        yaml.safe_dump(intrinsic_yaml, intrinsic_file, default_flow_style=False)
        intrinsic_file.close()

        # stereo calibration
        try:
            # NOTE:
            # `subprocess.run` not available in Py2, use `subprocess.Popen`
            # result = subprocess.run(
            #     ['. /app/scripts/{}.sh'.format(topic)],
            #     check=True,
            #     shell=True
            # )

            process = subprocess.Popen(". /app/scripts/{}.sh".format(topic), shell=True)

            # Capture the output and error streams
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                print("Script output:\n{}".format(stdout))
                print("Script error (if any):\n{}".format(stderr))
            else:
                raise subprocess.CalledProcessError(
                    process.returncode, cmd=topic, output=stderr
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
                    # Wait for the process to terminate
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Forcefully kill the process if it doesn't terminate
                    process.kill()

            print("calibrate stereo {} done".format(topic))


# Check calibrations are completed for next step
mono_ok = check_calibration("/data", "mono")
stereo_ok = check_calibration("/data", "stereo")
imu_ok = check_imu_calibration("/data")

if not mono_ok or not stereo_ok or not imu_ok:
    print("Calibration check failed. Aborting.")
    print("mono {}".format("OK" if mono_ok else "NG"))
    print("stereo {}".format("OK" if stereo_ok else "NG"))
    print("imu {}".format("OK" if imu_ok else "NG"))
    sys.exit(1)  # Exit with status code 1 indicating an error

# Generate virtual stereo configuration
generate_d2vins_configuration("/data")

# Calibrate virtual stereo
input_bag = os.path.join("/data", "stereo_depth_calibration.bag")
output_path = os.path.join(
    "/data", "virtual_stereo_calibration_{}".format(conf.virtual_fov)
)

photometric_mask_path = None
if conf.photometric_calibration_folder:
    photometric_mask_path = os.path.join(
        "/data", conf.photometric_calibration_folder)
fisheye_config = os.path.join("/data", "fisheye_cams.yaml")
step = 1
vsc.calibrate_virtual_stereo(
    input_bag,
    conf.virtual_fov,
    conf.virtual_width,
    conf.virtual_height,
    fisheye_config,
    output_path,
    step,
    photometric_mask_path,
    verbose=False,
)
