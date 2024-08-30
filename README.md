# omninxt-replica
Reproducing the OmniNxt project

## Background
This is a personal project that attempts to reproduce the opensourced [OmniNxt](https://github.com/HKUST-Aerial-Robotics/OmniNxt) project, which is released by the HKUST Aerial Robotics Group, their projects are already well documented and easy to follow through, please refer to their project for more details. Since it has been a while wanting to learn multi-camera platfrom for autonomous control system, so this repository simply serves as a learning memo along building the quadcam drone platform. However not a professional SLAM and control algorithm personnel, therefore I would not go into the detail explaining algorithms behind, since there may have some mis-understanding in the concept of the proposed algorithms, going through their [OmniNxt paper](https://arxiv.org/pdf/2403.20085) may be better way to understand.

AFAIK, the OmniNxt consists few necessary components to make the system to work.
* [**HW**] The quadcam drone platform
* [**SW**] Camera calibration pipeline
* [**SW**] The Omni-depth: a.k.a The D2SLAM, for generating surround depth information
* [**SW**] The Omni-Vins: a.k.a The D2SLAM, For self-localization
* [**SW**] Controls algorithms

## Build the OmniNxt platform

This is the multi-camera platform that all the software components are based on, therefore the choise of the each hardware parts are quite important, e.g. Camera module, camera lens, ESC, motor, flight controller. Please refer to the `Build your own OmniNxt` section for details, they already provided the information for the exact parts to use and where to get them.

The following is the platform I finally came up with

<img src="assets/omninxt_platform.png" width="90%">
