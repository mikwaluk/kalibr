# Instructions how to calibrate an Android smartphone with an april grid
This short set of instrictions assumes that you're using the app [Android Dataset Recorder](https://github.com/mikwaluk/android-dataset-recorder).

1. Firstly, with the app mentioned before, record a dataset to estimate the camera intrinsics and camera-imu extrinsics with it. This [video](https://www.youtube.com/watch?v=puNXsnrYWTY) gives a nice overview of the entire process.
2. After recording the data and transferring it to a PC, execute the following command to create a rosbag from the data:
```
rosrun kalibr kalibr_bagcreater --folder path/to/data --output-bag path/to/bag 
```
The bag can later be used either for calibration, or as an input to a VIO algorithm.

3. Having the bag, it’s possible to run kalibr to estimate the camera intrinsics.
4. 
At least for me, kalibr was unable to calibrate a single camera, therefore I used a trick and simulated having 2 cameras by just assigning the same image topic to the second camera. 

The [aprilgrid.yaml file](https://drive.google.com/file/d/1frx54DapXI0BB2fXSUyAQU6LKeduoX7Y/view) describes the calibration grid that I used. It was displayed on the screen of my PC. I followed the instructions from the[Kalibr Wiki](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) and at the end the aprilgrid configuration file `aprilgrid.yaml` had the following content:

```
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.026
tagSpacing: 0.2885
```
In addition, for the calibration command  I needed to adjust the `--aprox-sync` parameter after the calibration failed with a hint to decrease it.

The calibration command I used was
```
$ rosrun kalibr kalibr_calibrate_cameras --bag /path/to/bag --topics /cam0/image_raw /cam0/image_raw --models pinhole-radtan pinhole-radtan --target aprilgrid.yaml --approx-sync 0.0001
```
As mentioned before, I used a trick faking a second camera because the calibration failed for just one.

The calibration process resulted with a file `camchain-test.yaml` which contained the intrinsics of both cameras (which were the same, as only one camera was used in the reality) as well as the calculated baseline between them which was almost 0 (1e-7m).

Then, I was able to finally start the imu-camera calibration (again, simulating the second camera with input form the first one). I also needed to provide noise parameters for the IMU in the imu.yaml file, which I had to guess. I used similar values that had been already in use for VINS-Mobile (apart from the imu fequency which was known to be 50Hz from the dataset recorder):
```
accelerometer_noise_density: 0.006
accelerometer_random_walk: 0.0002
gyroscope_noise_density: 0.0004
gyroscope_random_walk: 4.0e-06
update_rate: 50.0
rostopic: "/imu"
```
The camera-imu calibration command had the form of:
```
$ rosrun kalibr kalibr_calibrate_imu_camera --target aprilgrid.yaml --cam path/to/cam/calibration/result --imu imu.yaml --bag /path/to/bag
```
This calibration process took a while, but I ended up with a precise estimate of the transformation between imu and camera.

The rest of this file comes from the original Kalibr repo and hasn't been modified.

![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

<!--*Ubuntu 14.04+ROS indigo*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-trusty/) *Ubuntu 16.04+ROS kinetic*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-xenial/)-->

## Introduction
Kalibr is a toolbox that solves the following calibration problems:

1. **Multiple camera calibration**: 
    intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view
1. **Visual-inertial calibration calibration (camera-IMU)**:
    spatial and temporal calibration of an IMU w.r.t a camera-system
1. **Rolling Shutter Camera calibration**:
    full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras


**Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.**

**For questions or comments, please open an issue on Github.**

## Installation

### Ubuntu 20.04

We've upgraded and fixed kalibr at ORI for 20.04. Please use our fork: `git clone https://github.com/ori-drs/kalibr.git --branch noetic-devel`.

- Use `rosdep` to install almost all required dependencies: `rosdep install --from-paths ./ -iry`.
- Then install the two missing runtime dependencies: `sudo apt install python3-wxgtk4.0 python3-igraph`
- Unittests are currently failing on 20.04 and thus deactivated on the buildserver.

## Tutorial: IMU-camera calibration
A video tutorial for the IMU-camera calibration can be found here:

[![alt text](https://user-images.githubusercontent.com/5337083/44033014-50208b8a-9f09-11e8-8e9a-d7d6d3c69d97.png)](https://m.youtube.com/watch?v=puNXsnrYWTY "imu cam calib")

(Credits: @indigomega)

## Authors
* Paul Furgale ([email](paul.furgale@mavt.ethz.ch))
* Hannes Sommer ([email](hannes.sommer@mavt.ethz.ch))
* Jérôme Maye ([email](jerome.maye@mavt.ethz.ch))
* Jörn Rehder ([email](joern.rehder@mavt.ethz.ch))
* Thomas Schneider ([email](schneith@ethz.ch))
* Luc Oth

## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

## Acknowledgments
This work is supported in part by the European Union's Seventh Framework Programme (FP7/2007-2013) under grants #269916 (V-Charge), and #610603 (EUROPA2).

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
