# ROVTIO: RObust Visual Thermal Inertial Odometry - For ROS2#
This repo contains a ros2 implementation of ROVTIO (tested on Humble, Ubuntu 22.04). 

The ROS2 implementation is heavily inspired by the ROVIO ROS2 implementation by suyash023 but with modifications for ROVTIO (Visual + Thermal Image Support). see suyash023's [ROVIO ROS2](https://github.com/suyash023/rovio).

The work of [ROVTIO](https://github.com/ntnu-arl/rovtio) is built on the original [ROVIO](https://github.com/ethz-asl/rovio). Check their GitHubs for further information and informative resources.


# Original ROVTIO README

## Install

ROVTIO requires no additional dependencies from the ones ROVIO requires. See the ROVIO install directions [below](#install-without-opengl-scene).

Note that it is recomended to install `kindr` [using catkin](https://github.com/ethz-asl/kindr#building-with-catkin).

## Setup and running
 
1. Install the dependencies for [ROVIO](#install-without-opengl-scene)
2. Configure the rovtio.info file and the .launch file. Examples are in `cfg/rovtio`.

### Running ROVTIO on the datsets considered in the master thesis

This is using the [`catkin tools`](https://catkin-tools.readthedocs.io/en/latest/index.html) to build the ros package. You can also build it with `catkin_make`.

1. Download the desired dataset from [here](https://drive.google.com/drive/folders/1FExxmw5FVcu1FAYibpvGxqSUa2cIfCmh).
2. Unzip the zipfile.
2. Build ROVTIO `catkin build rovtio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=OFF -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=25`
4. Run ROVTIO `roslaunch rovtio rovtio.launch`
5. Play the bags. Enter the dataset folder and `rosbag play *.bag`
6. Optional: Open `rviz` and monitor `/rovtio/odometry`. You can compare it to the near ground truth result from LOAM at the topic`/aft_mapped_to_init_CORRECTED`.

## Using more cameras

ROVTIO can use several cameras out of the box, but needs to be configured accordingly. They can be either thermal or visual cameras, but the related parameters in the `.info` file should be chosen accordingly.

1. Edit the launch file: You need to add a `camera_topicX` where the each camera needs to be designated an id `X` which is numbers starting at 0 and increasing by 1. If the timing offset between the camera and the IMU is known, you can specify it with the `camX_offset` where `X` is the camera ID.
2. Edit the .info file: There is several parameters which needs to be specified per camera. This is because you typically want different parameters for thermal cameras and visual cameras. Look in the example file and duplicate all the parameters that ends with a number(eg. `minSTScoreForModalitySelectionX`) and change the number `X` To the camera ID.
3. Rebuild with the new number of cameras in the `-DROVIO_NCAM` build option.

The camera streams does not have to be synchronized and ROVTIO will align the m if they arrive out of order.

## License

This repo inherits the license from ROVIO. For details see `LICENSE`.

# Orignal ROVIO README #

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (as submodule, use "git submodule update --init --recursive")

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
