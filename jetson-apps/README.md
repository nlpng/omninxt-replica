# jetson-apps
> Applications run on the Jetson Orin of the OmniNxt quadcam platform
## Background
This is mainly based on the [D2SLAM](https://github.com/HKUST-Aerial-Robotics/D2SLAM.git) introduced by HKUST Aerial Robotics Group. Currently used for personal learning purposes in order to understand how the D2SLAM works, therefore there might have detail descriptions missing in the documents, like how to build the platform, connecting all the devices, or the wiring diagram, since they all have been completed at the background.

## Intro
This directory contains the applications that would be installed on the Jeston Orin SBC, and the applications are dockerized for smooth reproduction. It seems that there is already docker containers built by the HKUST Aerial Robotics Group for the OmniNxt D2SLAM, since I could not find it, I will build the docker by myself, also modify the source code along the way of learing D2SLAM to fit my own needs.

The applications are communicated(?) using ROS nodes, and ROS 1 noetic version is used in this project. There are 3 main components:
* [MAVROS](https://github.com/mavlink/mavros) - Getting the telemetry data from PX4 through [mavlink](https://mavlink.io/en/) protocol
* [Depthai-ros](https://github.com/luxonis/depthai-ros) - To interface with the OAK-FFC 4P board that could connect up to 4 FFC camera modules.
* D2SLAM - The main SLAM algorithm happens here.

## How to use

To make things clear, these applications should either be built on the Jeston, or pull the pre-built docker images on the Jetson. Building the docker images on Jetson may take a while, so if pre-built images are known do pull them to the Jetson and run directly from that.

### Depthai and PX4
The cameras are handled by Depthai [OAK-FFC 4P](https://shop.luxonis.com/products/oak-ffc-4p?srsltid=AfmBOooGq76megpT-8kbCgfa1ZkgPE8glZ-CYYA-K48_rHUATdT-Ki9Y) board, and camera frames are published through ROS. The follwing references are consulted for the hardware or software sync of all 4 cameras [1](https://discuss.luxonis.com/d/934-ffc-4p-hardware-synchronization/3), [2](https://docs.luxonis.com/hardware/platform/deploy/frame-sync/).

As well as the PX4 telemetry data are handled by MAVROS then published to ROS. In order to setup the connection between the PX4 enabled flight controller with Jetson, please following the PX4 documentation on [companion computer](https://docs.px4.io/main/en/companion_computer/).

* Change directory to the top of the `comm` directory
```
cd to/the/comm/folder
```

* Build the image, this will build everything that required to run the depthai camera and Mavros on Jetson.
```
docker build -f ./Dockerfile -t comm:noetic .
```
or using `Makefile`

```
make comm-noetic
```

After building the image, there are scripts prepared to run the docker image in the `scripts`. First make the scripts executable, the run the scripts to launch necessary node.

```
chmod +x scripts/*
```

Start the OAK-FFC 4P camera node
```
./scripts/ffc_4p.sh
```

Start MAVROS node
```
./scripts/mavros.sh
```

For other operations, such as start nodes for preparing calibration data, and recording the data as ROS bags.
```
./scripts/calibr.sh
./scripts/robag.sh
```


### D2SLAM
The docker images are based on `nvcr.io/nvidia/l4t-jetpack:r35.4.1`, in order to match the jetpack version used during the testing is in 35.4.1. Match other version of jetpack maybe possible, but not tested.

* Change directory to the top of the `d2slam` directory
```
cd to/the/d2slam/folder
```

* Build the base image. The base image installs the necessary components for D2SLAM.

```
make d2slam-base
```
* Build the application image. The application image installs D2SLAM.

```
make d2slam-noetic
```
