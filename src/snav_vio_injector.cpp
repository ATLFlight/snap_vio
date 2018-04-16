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
#include "snap_vio/snav_vio_injector.hpp"

SnavInterface::SnavInterface(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(pnh)
{

  vio_odom_subscriber_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "vio/odometry", 10);
  vio_states_subscriber_ = new message_filters::Subscriber<snap_msgs::InternalStates>(nh_, "vio/internal_states", 10);
  vio_points_subscriber_ = new message_filters::Subscriber<snap_msgs::MapPointArray>(nh_, "vio/map_points", 10);

  vio_sync_ = new message_filters::TimeSynchronizer<nav_msgs::Odometry,
                                                    snap_msgs::InternalStates,
                                                    snap_msgs::MapPointArray>(*vio_odom_subscriber_,
                                                                              *vio_states_subscriber_,
                                                                              *vio_points_subscriber_, 10);
  vio_sync_->registerCallback(boost::bind(&SnavInterface::VioCallback, this, _1, _2, _3));

  if(sn_get_flight_data_ptr(sizeof(SnavCachedData),&cached_data_)!=0){
    ROS_ERROR("Error getting cached data.\n");
  };
  sn_update_data();
  last_sn_update_ = ros::Time::now();


  nh_.param("/use_sim_time", simulation_, false);
  simulation_ = false;
  if(!simulation_)
    GetDSPTimeOffset();
  else
  {
    dsp_offset_in_ns_  = 0;
  }
}


SnavInterface::~SnavInterface() {
}


void SnavInterface::GetDSPTimeOffset()
{
  // get the adsp offset.
  int64_t dsptime;
#ifdef QC_SOC_TARGET_APQ8096
  static const char qdspTimerTickPath[] = "/sys/kernel/boot_slpi/qdsp_qtimer";
#endif
#ifdef QC_SOC_TARGET_APQ8074
  static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
#endif
  char qdspTicksStr[20] = "";

  static const double clockFreq = 1 / 19.2;
  FILE * qdspClockfp = fopen( qdspTimerTickPath, "r" );
  fread( qdspTicksStr, 16, 1, qdspClockfp );
  uint64_t qdspTicks = strtoull( qdspTicksStr, 0, 16 );
  fclose( qdspClockfp );

  dsptime = (int64_t)( qdspTicks*clockFreq*1e3 );

  //get the apps proc timestamp;
  int64_t appstimeInNs;
  struct timespec t;
  clock_gettime( CLOCK_REALTIME, &t );

  uint64_t timeNanoSecMonotonic = (uint64_t)(t.tv_sec) * 1000000000ULL + t.tv_nsec;
  appstimeInNs = (int64_t)timeNanoSecMonotonic;

  // now compute the offset.
  dsp_offset_in_ns_  = appstimeInNs - dsptime;

  ROS_INFO_STREAM("DSP offset: " <<   dsp_offset_in_ns_ << " ns");
}


void SnavInterface::VioCallback(const nav_msgs::OdometryConstPtr& odom,
                                const snap_msgs::InternalStatesConstPtr& states,
                                const snap_msgs::MapPointArrayConstPtr& points)
{
  //ROS_INFO_STREAM("Received messages for VIO timestamp= " << odom->header.stamp);

  if(!states->snav_mode){
    ROS_ERROR("snav_vio_injector requires snap_vio to run in snav_mode (param)");
    return;
  }

  static uint32_t cntr = 0;
  VioRpcData vio_rpc;
  vio_rpc.cntr = cntr;
  vio_rpc.timestamp_s = (states->header.stamp + states->time_alignment.data).toSec();
  vio_rpc.filter_diverged = 0;
  if (states->pose_quality == -2) {
    vio_rpc.filter_diverged = 1;
  }
  vio_rpc.pose_tracking_quality = states->pose_quality;
  vio_rpc.time_alignment = states->time_alignment.data.toSec();
  vio_rpc.error_code = states->error_code;

  // pose and velocity
  tf2::Quaternion bq(odom->pose.pose.orientation.x,
                     odom->pose.pose.orientation.y,
                     odom->pose.pose.orientation.z,
                     odom->pose.pose.orientation.w);
  tf2::Matrix3x3 br(bq);
  std::copy(&br[0][0], &br[0][0]+3, &vio_rpc.body_pose[0][0]);
  std::copy(&br[1][0], &br[1][0]+3, &vio_rpc.body_pose[1][0]);
  std::copy(&br[2][0], &br[2][0]+3, &vio_rpc.body_pose[2][0]);
  vio_rpc.body_pose[0][3] = odom->pose.pose.position.x;
  vio_rpc.body_pose[1][3] = odom->pose.pose.position.y;
  vio_rpc.body_pose[2][3] = odom->pose.pose.position.z;
  std::copy(&odom->pose.covariance[0], &odom->pose.covariance[0]+36,
      &vio_rpc.err_cov_pose[0][0]);
  vio_rpc.velocity[0] = odom->twist.twist.linear.x;
  vio_rpc.velocity[1] = odom->twist.twist.linear.y;
  vio_rpc.velocity[2] = odom->twist.twist.linear.z;
  vio_rpc.angular_velocity[0] = odom->twist.twist.angular.x;
  vio_rpc.angular_velocity[1] = odom->twist.twist.angular.y;
  vio_rpc.angular_velocity[2] = odom->twist.twist.angular.z;
  std::copy(&odom->twist.covariance[0], &odom->twist.covariance[0]+3,
      &vio_rpc.err_cov_velocity[0][0]);
  std::copy(&odom->twist.covariance[6], &odom->twist.covariance[6]+3,
      &vio_rpc.err_cov_velocity[1][0]);
  std::copy(&odom->twist.covariance[12], &odom->twist.covariance[12]+3,
      &vio_rpc.err_cov_velocity[2][0]);

  // gravity
  tf2::Quaternion gq(states->gravity_camera_pose.rotation.x,
                     states->gravity_camera_pose.rotation.y,
                     states->gravity_camera_pose.rotation.z,
                     states->gravity_camera_pose.rotation.w);
  tf2::Matrix3x3 gr(gq);
  std::copy(&gr[0][0], &gr[0][0]+3, &vio_rpc.gravity_camera_pose[0][0]);
  std::copy(&gr[1][0], &gr[1][0]+3, &vio_rpc.gravity_camera_pose[1][0]);
  std::copy(&gr[2][0], &gr[2][0]+3, &vio_rpc.gravity_camera_pose[2][0]);
  vio_rpc.gravity_camera_pose[0][3] = states->gravity_camera_pose.translation.x;
  vio_rpc.gravity_camera_pose[1][3] = states->gravity_camera_pose.translation.y;
  vio_rpc.gravity_camera_pose[2][3] = states->gravity_camera_pose.translation.z;
  vio_rpc.gravity[0] = states->gravity.x;
  vio_rpc.gravity[1] = states->gravity.y;
  vio_rpc.gravity[2] = states->gravity.z;
  std::copy(&states->err_cov_gravity[0], &states->err_cov_gravity[0]+9,
      &vio_rpc.err_cov_gravity[0][0]);

  // accel/gyro internal states
  vio_rpc.gyro_bias[0] = states->gyro_bias.x;
  vio_rpc.gyro_bias[1] = states->gyro_bias.y;
  vio_rpc.gyro_bias[2] = states->gyro_bias.z;
  vio_rpc.accel_bias[0] = states->accel_bias.x;
  vio_rpc.accel_bias[1] = states->accel_bias.y;
  vio_rpc.accel_bias[2] = states->accel_bias.z;
  tf2::Quaternion aq(states->tf_imu_camera.rotation.x,
                     states->tf_imu_camera.rotation.y,
                     states->tf_imu_camera.rotation.z,
                     states->tf_imu_camera.rotation.w);
  tf2::Matrix3x3 ar(aq);
  std::copy(&ar[0][0], &ar[0][0]+9, &vio_rpc.R_accel_camera[0][0]);
  vio_rpc.t_accel_camera[0] = states->tf_imu_camera.translation.x;
  vio_rpc.t_accel_camera[1] = states->tf_imu_camera.translation.y;
  vio_rpc.t_accel_camera[2] = states->tf_imu_camera.translation.z;

  // map points
  vio_rpc.num_points = points->map_points.size();

  // TODO -- add vio points to snav for snav logs
//MAX_VISLAM_TRACKING_PTS
//  for (int ii = 0; ii < vio_rpc.num_points; ++ii) {
//    vio_rpc.point[ii].id = points->map_points[ii].id;
//    vio_rpc.point[ii].score = points->map_points[ii].score;
//    vio_rpc.point[ii].pix_loc[0] = points->map_points[ii].pixel_location[0];
//    vio_rpc.point[ii].pix_loc[1] = points->map_points[ii].pixel_location[1];
//    vio_rpc.point[ii].tsf[0] = points->map_points[ii].point.x;
//    vio_rpc.point[ii].tsf[1] = points->map_points[ii].point.y;
//    vio_rpc.point[ii].tsf[2] = points->map_points[ii].point.z;
//    std::copy(&points->map_points[ii].point_covariance[0],
//        &points->map_points[ii].point_covariance[0]+9,
//        &vio_rpc.point[ii].err_cov_tsf[0][0]);
//    vio_rpc.point[ii].depth = points->map_points[ii].depth;
//    vio_rpc.point[ii].depth_err_std_dev = points->map_points[ii].depth_std_dev;
//  }

  // sensor statistics
  // TODO
  vio_rpc.frame_timestamp_ns = 0;
  vio_rpc.frame_id = 0;
  vio_rpc.frame_cntr = 0;
  vio_rpc.frame_drop_cntr = 0;
  vio_rpc.accel_sample_cntr = 0;
  vio_rpc.accel_sample_drop_cntr = 0;
  vio_rpc.gyro_sample_cntr = 0;
  vio_rpc.gyro_sample_drop_cntr = 0;

  sn_send_vio_data(vio_rpc);
}

