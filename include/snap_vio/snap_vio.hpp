/****************************************************************************
 *   Copyright (c) 2018 Michael Shomin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/
#ifndef _SNAP_VIO_H_
#define _SNAP_VIO_H_

#include <iostream>
#include <chrono>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <snap_msgs/ExposureTimes.h>
#include <snap_msgs/InternalStates.h>
#include <snap_msgs/MapPointArray.h>
#include <snap_msgs/ImuArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/static_transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_subscriber.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <mvVISLAM.h>

class SnapVio
{
public:
  typedef struct {
    float32_t   readoutTime;
    float32_t   tbc[3];
    float32_t   ombc[3];
    float32_t   delta;
    float32_t   std0Tbc[3];
    float32_t   std0Ombc[3];
    float32_t   std0Delta;
    float32_t   accelMeasRange;
    float32_t   gyroMeasRange;
    float32_t   stdAccelMeasNoise;
    float32_t   stdGyroMeasNoise;
    float32_t   stdCamNoise;
    float32_t   minStdPixelNoise;
    float32_t   failHighPixelNoiseScaleFactor;
    float32_t   logDepthBootstrap;
    bool        useLogCameraHeight;
    float32_t   logCameraHeightBootstrap;
    bool        noInitWhenMoving;
    float32_t   limitedIMUbWtrigger;
    std::string staticMaskFilename;
    float32_t   gpsImuTimeAlignment;
    float32_t   tba[3];
  } InitParams;

  /**
   * Constructor.
   * @param nh
   *   nodehandle to intialize the node.
   * @param pnh
   *   private namespace nodehandle for this node
   */
  SnapVio(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~SnapVio();

  bool Start();
  void Stop();

private:

  bool CheckInitVio();
  bool InitializeVio();

  void PublishVioData(mvVISLAMPose& vio_pose, int64_t vio_frame_id,
                      ros::Time vio_timestamp);
  void PublishMapPoints(std::vector<mvVISLAMMapPoint>& vio_points, int64_t vio_frame_id,
                      ros::Time image_timestamp);

  void ImuRawCallback(const snap_msgs::ImuArrayConstPtr& msg);

  void SyncedCallback(const sensor_msgs::ImageConstPtr& msg,
                      const snap_msgs::ExposureTimesConstPtr& exp_msg);

  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Publisher vio_pose_publisher_;
  ros::Publisher vio_odom_publisher_;
  ros::Publisher vio_states_publisher_;
  ros::Publisher vio_points_publisher_;
  ros::Publisher vio_cloud_publisher_;

  ros::Subscriber imu_raw_subscriber_;
  ros::Subscriber cinfo_subscriber_;

  //public namespace nodehandle
  ros::NodeHandle nh_;
  //private namespace nodehandle
  ros::NodeHandle pnh_;

  image_transport::ImageTransport im_trans_;
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber img_sub_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, 
    snap_msgs::ExposureTimes
    > ExpoSyncPolicy;

  message_filters::Subscriber<snap_msgs::ExposureTimes> expo_sub_;
  message_filters::Synchronizer< ExpoSyncPolicy > sync;

  ros::Time latest_imu_timestamp_;
  ros::Duration latest_time_alignment_;

  mvCameraConfiguration mv_camera_config_;
  InitParams vislam_params_;
  mvVISLAM* vislam_ptr_;

  bool running_;
  bool initialized_;
  bool no_frames_yet_;

  bool got_camera_cal_;
  bool got_imu_frame_id_;
  bool got_camera_frame_id_;
  bool got_imu_cam_tf_;

  std::string camera_frame_;
  std::string imu_frame_;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener * tfListener;

  bool snav_mode_;
};

#endif
