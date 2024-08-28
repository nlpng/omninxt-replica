#!/usr/bin/env python

# photometric calibration
photometric_calibration_path = None

# ROS bag topics
image_topic = "/oak_ffc_4p/assemble_image/compressed"
imu_topic = "/mavros/imu/data_raw"

# I/O
rosbag_path = "./data/quadcam_calib.bag"
output_path = "./data/extracted_output/"

# virtual stereo parameters
virtual_fov = 180
virtual_width = 320
virtual_height = 240

step_dict = {0: 0, 1: 1, 2: 2, 4: 3, 8: 4, 9: 5, 3: 6, 6: 7, 12: 8}
rosbag_name = ["ERROR_BAG_NAME_Shouldnot_exsist",
               "CAM_A", "CAM_B", "CAM_C", "CAM_D",
               "CAM_D-CAM_A", "CAM_A-CAM_B",
               "CAM_B-CAM_C", "CAM_C-CAM_D"]
camera_topics = ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]
current_step = 1
