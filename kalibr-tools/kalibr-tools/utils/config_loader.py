#!/usr/bin/env python
import numpy as np
import yaml

from utils.fisheye_undist import FisheyeUndist
from utils.stereo_gen import StereoGen


def pinholeIntrinsicToCameraMatrix(int):
    K0 = np.array([
                [int[0], 0, int[2]],
                [0, int[1], int[3]],
                [0, 0, 1]])
    return K0


def initStereoFromConfig(config_file, stereo_gen, force_width=None):
    print("Init stereo with config", config_file)
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
        size = config["cam0"]["resolution"]
        K0 = pinholeIntrinsicToCameraMatrix(config["cam0"]['intrinsics'])
        K1 = pinholeIntrinsicToCameraMatrix(config["cam1"]['intrinsics'])
        print("Debug K1", K1)
        if force_width is not None:
            K0 *= force_width/size[0]
            K1 *= force_width/size[0]
            size = (force_width, int(force_width*size[1]/size[0]))
            print("Force width to", force_width, "\nK0\n", K0, "\nK1\n", K1)
        D0 = np.array(config["cam0"]['distortion_coeffs'], dtype=np.float)
        D1 = np.array(config["cam1"]['distortion_coeffs'], dtype=np.float)
        T = np.array(config["cam1"]["T_cn_cnm1"])
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        stereo_gen.initRectify(K0, D0, K1, D1, (size[0], size[1]), R, t)


def LoadFisheyeParameter(
        config_file, config_stereos=[], fov=190, width=600, height=300):
    print("Loading config from {}".format(config_file))
    undists = []
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
        for v in config:
            print(v)
            intrinsics = config[v]['intrinsics']
            distortion_coeffs = config[v]['distortion_coeffs']
            xi = intrinsics[0]
            gamma1 = intrinsics[1]
            gamma2 = intrinsics[2]
            u0 = intrinsics[3]
            v0 = intrinsics[4]
            K = np.array(
                [[gamma1, 0, u0],
                 [0, gamma2, v0],
                 [0, 0, 1]])
            D = np.array(distortion_coeffs)
            try:
                T = np.array(config[v]['T_cam_imu'])
            except Exception as e:
                T = np.eye(4)
                print("{} set T to {}".format(e, T))

            undist = FisheyeUndist(
                K, D, xi, fov=fov, width=width, height=height, extrinsic=T)
            undists.append(undist)
    gens = [
            StereoGen(
                undists[0], undists[1],
                cam_idx_a=0, cam_idx_b=1, hitnet_model=None, is_rgb=True),
            StereoGen(
                undists[1], undists[2],
                cam_idx_a=1, cam_idx_b=2, hitnet_model=None, is_rgb=True),
            StereoGen(
                undists[2], undists[3],
                cam_idx_a=2, cam_idx_b=3, hitnet_model=None, is_rgb=True),
            StereoGen(
                undists[3], undists[0],
                cam_idx_a=3, cam_idx_b=0, hitnet_model=None, is_rgb=True)]

    for i in range(len(config_stereos)):
        initStereoFromConfig(config_stereos[i], gens[i], force_width=width)

    return gens, True
