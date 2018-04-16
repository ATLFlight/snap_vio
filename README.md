## snap_vio - ROS implementation of MV VISLAM
snap_vio wraps mvVISLAM from the machine vision library, providing a ROS interface.

## Dependencies
- [snap_cam_ros](https://github.com/ATLFlight/snap_cam_ros) ([snap_cam_manager](https://github.com/ATLFlight/snap_cam_manager) as submodule)
- [snap_msgs](https://github.com/ATLFlight/snap_msgs)
- [qflight_descriptions](https://github.com/ATLFlight/qflight_descriptions)
- [snap_cpa](https://github.com/ATLFlight/snap_cpa)
- [snap_imu](https://github.com/ATLFlight/snap_imu)

## Run It!
snap_vio needs a few things to work:
- image msgs from a camera (sensor_msgs/Image via image_transport to "image_raw")
- camera calibration (sensor_msgs/CameraInfo on "camera_info" topic) -- note that the "standalone.launch" spoofs this calibration using the approximate fisheye model params because ROS cameras don't have full fisheye support.  If this changes in the future, this package will already be compatible
- exposure time messages for image frames (snap_msgs/ExposureTimes) -- published by snap_cam_ros
- /imu -> camera_frame transform -- provided by qflight_descriptions
- imu_messages (snap_msgs/ImuArray) -- published by snap_imu
- snap_cpa can optionally be run alongside snap_cam_ros to get good exposure/gain for tracking features

Before launching, something needs to be providing imu data from the DSP.  This can be either snav or the standalone imu server.  If you are not running snav, first run:
```bash
imu_app -s 2 &  # if running on 8096, you will also need to add the -p 10 option to use the correct imu
```
This will provide 500 Hz imu data for snap_imu to publish and ros messages.  You can then start snap_vio with:
```bash
roslaunch snap_vio standalone.launch
```
If you running on an 8096 board:
```bash
roslaunch snap_vio standalone.launch 8074:=false
```
This example launch file with run all the components necessary for snap_vio.

## Feature Point Image Overlay

Included in this package is scripts/vislam_point_overlay.py.  This script will draw circle for tracked feature points on the input image and publish this image on "/downward/image_overlay".  You can use rqt_image_view to look at this output.  It is recommended to run this script off-target on a workstation.  To minimize stutter/image delays, it is also recommended to use compressed image transport.  Using launch/vislam_pt_overlay.launch will subscribe to the compressed image, republish a decompressed image, run the overlay script, and start rqt_image_view.

## SNAV Compatibility

Snap_vio cannot run alongside snav_dft_vio_app, as snap_cam_ros needs to consume the downward image frames for snap_vio.  If, however, you have SNAV version 1.2.59 or greater, the output of snap_vio can be "injected" into SNAV instead of snav_dft_vio_app.  Make sure to disable snav_dft_vio_app before running.  You can use launch/snav_vio.launch for running snav_vio and the injector node.  Note that the injector node will not build unless you have SNAV version 1.2.59 or greater.

### Notes
Also try changing the vehicle arg in the standalone.launch from "none" to either "ugv_example" or "dragonfly"
