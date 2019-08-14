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
#include "snap_vio/snap_vio.hpp"

SnapVio::SnapVio(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh),
    pnh_(pnh),
    im_trans_(nh_),
    img_sub_(im_trans_, "image_raw", 1),
    expo_sub_(nh_, "exposure_times", 1),
    sync( ExpoSyncPolicy( 10 ), img_sub_, expo_sub_ )
{
  running_ = false;
  initialized_ = false;
  no_frames_yet_ = true;
  vislam_ptr_ = nullptr;

  got_camera_cal_ = false;
  got_imu_frame_id_ = false;
  got_camera_frame_id_ = false;
  got_imu_cam_tf_ = false;

  pnh_.param("snav_mode", snav_mode_, false);

  pnh_.param("noInitWhenMoving",    vislam_params_.noInitWhenMoving,    true);
  pnh_.param("limitedIMUbWtrigger", vislam_params_.limitedIMUbWtrigger, 200.0f);
  pnh_.param("gpsImuTimeAlignment", vislam_params_.gpsImuTimeAlignment, 0.0f);
  pnh_.param("readoutTime",         vislam_params_.readoutTime,         0.0f);
  pnh_.param("delta",               vislam_params_.delta,               0.002f);
  pnh_.param("std0Delta",           vislam_params_.std0Delta,           0.001f);
  pnh_.param("accelMeasRange",      vislam_params_.accelMeasRange,      156.0f);
  pnh_.param("gyroMeasRange",       vislam_params_.gyroMeasRange,       34.0f);
  pnh_.param("stdAccelMeasNoise",   vislam_params_.stdAccelMeasNoise,   0.316f);
  pnh_.param("stdGyroMeasNoise",    vislam_params_.stdGyroMeasNoise,    1e-2f);
  pnh_.param("stdCamNoise",         vislam_params_.stdCamNoise,         100.0f);
  pnh_.param("minStdPixelNoise",    vislam_params_.minStdPixelNoise,    0.5f);
  pnh_.param("logDepthBootstrap",   vislam_params_.logDepthBootstrap,   0.0f);
  pnh_.param("useLogCameraHeight",  vislam_params_.useLogCameraHeight,  false);
  pnh_.param("logCameraHeightBootstrap",
            vislam_params_.logCameraHeightBootstrap,-3.22f);
  pnh_.param("failHighPixelNoiseScaleFactor",
            vislam_params_.failHighPixelNoiseScaleFactor, 1.6651f);
  pnh_.param<std::string>("staticMaskFilename", vislam_params_.staticMaskFilename,"na");
  std::vector<float> std0Tbc;
  pnh_.param("std0Tbc", std0Tbc, {0.005, 0.005, 0.005});
  vislam_params_.std0Tbc[0] = std0Tbc[0];
  vislam_params_.std0Tbc[1] = std0Tbc[1];
  vislam_params_.std0Tbc[2] = std0Tbc[2];

  std::vector<float> std0Ombc;
  pnh_.param("std0Ombc", std0Ombc, {0.04, 0.04, 0.04});
  vislam_params_.std0Ombc[0] = std0Ombc[0];
  vislam_params_.std0Ombc[1] = std0Ombc[1];
  vislam_params_.std0Ombc[2] = std0Ombc[2];

  std::vector<float> tba;
  pnh_.param("tba", tba, {0.0, 0.0, 0.0});
  vislam_params_.tba[0] = tba[0];
  vislam_params_.tba[1] = tba[1];
  vislam_params_.tba[2] = tba[2];

  latest_time_alignment_ = ros::Duration(vislam_params_.delta);
  ROS_INFO_STREAM("Initial Time alignment: " << latest_time_alignment_ );

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  vio_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("vio/pose",1);
  vio_odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("vio/odometry",1);
  vio_states_publisher_ = nh_.advertise<snap_msgs::InternalStates>("vio/internal_states",1);
  vio_points_publisher_ = nh_.advertise<snap_msgs::MapPointArray>("vio/map_points",1);
  vio_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("vio/point_cloud",1);

  imu_raw_subscriber_ = nh_.subscribe("imu_raw_array", 1000, &SnapVio::ImuRawCallback, this);
  cinfo_subscriber_ = nh_.subscribe("camera_info", 5, &SnapVio::CameraInfoCallback, this);

  sync.registerCallback( boost::bind( &SnapVio::SyncedCallback, this, _1, _2 ) );
}


SnapVio::~SnapVio() {
  //stop the vislam engine.
  if (vislam_ptr_ != nullptr) {
    mvVISLAM_Deinitialize( vislam_ptr_ );
    vislam_ptr_ = nullptr;
  }
}


bool SnapVio::Start() {
  std::cout << "Start" << std::endl;
  running_=true;
  return true;
}


void SnapVio::Stop() {
  running_=false;
}

void SnapVio::SyncedCallback(const sensor_msgs::ImageConstPtr& msg,
                             const snap_msgs::ExposureTimesConstPtr& expo_msg)
{

  if (!initialized_) {
    if(!CheckInitVio())
      return;
  }

  if (no_frames_yet_) {
    no_frames_yet_ = false;
    return;
  }

  static uint32_t frame_number_last = 0;
  if (frame_number_last != 0)
  {
    // The diff should be 1, anything greater means we dropped images
    if (msg->header.seq - frame_number_last != 1) {
      ROS_ERROR_STREAM("[VISLAM] Dropped Image Frame current: " << msg->header.seq <<
	    ", last: " << frame_number_last);
    }
  }
  frame_number_last = msg->header.seq;

  // TODO: smarter blocking here / is this necessary in newest MV?
  // Now wait for IMU samples to catch up
  while (latest_imu_timestamp_ < (expo_msg->center_of_exposure + latest_time_alignment_)) {
    ros::Duration(0.001).sleep();
  }

  // Process and publish
  mvVISLAM_AddImage(vislam_ptr_, expo_msg->center_of_exposure.toNSec(), &msg->data[0]);
  mvVISLAMPose pose = mvVISLAM_GetPose(vislam_ptr_);
  PublishVioData(pose, msg->header.seq, expo_msg->center_of_exposure);

  int num_points = mvVISLAM_HasUpdatedPointCloud(vislam_ptr_);
  std::vector<mvVISLAMMapPoint> map_points(num_points, {0});
  int num_received = mvVISLAM_GetPointCloud(vislam_ptr_, map_points.data(), num_points);
  PublishMapPoints(map_points, msg->header.seq, expo_msg->center_of_exposure);
}

void SnapVio::ImuRawCallback(const snap_msgs::ImuArrayConstPtr& msg) {
  if(!got_imu_frame_id_){
    imu_frame_ = msg->header.frame_id;
    got_imu_frame_id_ = true;
  }

  // Wait for the camera to start up before we do anything
  if (no_frames_yet_) {
    return;
  }

  static uint32_t sequence_number_last = 0;
  for (int ii = 0; ii < msg->imu_samples.size(); ++ii) {
    if (sequence_number_last != 0)
    {
      // The diff should be 1, anything greater means we dropped samples
      if (msg->sequence_numbers[ii] - sequence_number_last != 1) {
        ROS_ERROR_STREAM("[VISLAM] Dropped IMU samples current: " << msg->sequence_numbers[ii] <<
                       ", last: " << sequence_number_last);
      }
    }
    sequence_number_last = msg->sequence_numbers[ii];

    int64_t imu_timestamp_ns = msg->imu_samples[ii].header.stamp.toNSec();

    mvVISLAM_AddAccel(vislam_ptr_, imu_timestamp_ns,
                      msg->imu_samples[ii].linear_acceleration.x,
                      msg->imu_samples[ii].linear_acceleration.y,
                      msg->imu_samples[ii].linear_acceleration.z);
    mvVISLAM_AddGyro(vislam_ptr_, imu_timestamp_ns,
                     msg->imu_samples[ii].angular_velocity.x,
                     msg->imu_samples[ii].angular_velocity.y,
                     msg->imu_samples[ii].angular_velocity.z);

    // Record the latest timestamp
    // This is used to verify that all IMU samples up to the image have arrived
    latest_imu_timestamp_ = msg->imu_samples[ii].header.stamp;
  }
}

void SnapVio::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
  if(got_camera_cal_)
    return;

  mv_camera_config_.pixelWidth = msg->width;
  mv_camera_config_.pixelHeight = msg->height;
  mv_camera_config_.memoryStride = msg->width;

  mv_camera_config_.principalPoint[0] = msg->K[2];
  mv_camera_config_.principalPoint[1] = msg->K[5];
  mv_camera_config_.focalLength[0] = msg->K[0];
  mv_camera_config_.focalLength[1] = msg->K[4];

  mv_camera_config_.uvOffset = 0;

  for(int i=0; i < msg->D.size(); i++)
    mv_camera_config_.distortion[i] = msg->D[i];

  if(msg->distortion_model == "fisheye"){
    mv_camera_config_.distortionModel = 10;
    ROS_INFO("[VISLAM] Using fisheye camera model (mv model 10)");
  }
  else if(msg->distortion_model == "plumb_bob"){
    mv_camera_config_.distortionModel = 5;
    ROS_INFO("[VISLAM] Using plumb bob camera model (mv model 5)");
  }
  else{
    ROS_FATAL("[VISLAM] Unsupported Camera Model, exitting");
    exit(-1);
  }

  got_camera_cal_ = true;

  camera_frame_ = msg->header.frame_id;
  got_camera_frame_id_ = true;
}


bool SnapVio::CheckInitVio(){
  if(!got_camera_cal_){
    ROS_WARN_THROTTLE(1, "[VISLAM] Haven't received camera_info messages; can't init yet");
    return false;
  }
  if(!got_imu_frame_id_){
    ROS_WARN_THROTTLE(1, "[VISLAM] Haven't received imu messages; can't init yet");
    return false;
  }
  if(!got_camera_frame_id_){
    ROS_WARN_THROTTLE(1, "[VISLAM] Haven't received images messages; can't init yet");
    return false;
  }

  if(!got_imu_cam_tf_){
    geometry_msgs::TransformStamped imu_to_cam;
    try{
      imu_to_cam = tfBuffer.lookupTransform(imu_frame_, camera_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM_THROTTLE(1, "[VISLAM] Couldn't lookup " << imu_frame_ <<
                               " to " << camera_frame_ << " transform; can't init yet");
      ROS_WARN_STREAM_THROTTLE(1,"tf exception: " << ex.what());
      return false;
    }

    ROS_INFO_STREAM("[VISLAM] Got " << imu_frame_ << " to " << camera_frame_ <<
                    " transform; Initializing VISLAM");
    ROS_INFO_STREAM("[VISLAM] (from tf) tbc:\n" << imu_to_cam.transform.translation);

    double ang, aa_x, aa_y, aa_z, nmag;
    nmag = sqrt(1-(imu_to_cam.transform.rotation.w*imu_to_cam.transform.rotation.w));
    ang = 2 * acos(imu_to_cam.transform.rotation.w);
    aa_x = imu_to_cam.transform.rotation.x / nmag;
    aa_y = imu_to_cam.transform.rotation.y / nmag;
    aa_z = imu_to_cam.transform.rotation.z / nmag;

    double ombc[3];
    ombc[0] = aa_x*ang;
    ombc[1] = aa_y*ang;
    ombc[2] = aa_z*ang;

    ROS_INFO_STREAM("[VISLAM] (from tf) ombc:\n" <<
                    "X: " << ombc[0] << "\n" <<
                    "Y: " << ombc[1] << "\n" <<
                    "Z: " << ombc[2] << "\n");


    vislam_params_.tbc[0] = imu_to_cam.transform.translation.x;
    vislam_params_.tbc[1] = imu_to_cam.transform.translation.y;
    vislam_params_.tbc[2] = imu_to_cam.transform.translation.z;

    vislam_params_.ombc[0] = ombc[0];
    vislam_params_.ombc[1] = ombc[1];
    vislam_params_.ombc[2] = ombc[2];

    delete tfListener;
    got_imu_cam_tf_ = true;
  }

  try {
    return InitializeVio();
  }
  catch (std::runtime_error& e) {
    ROS_ERROR_STREAM("Error initializing mvVISLAM!");
    return false;
  }
}


bool SnapVio::InitializeVio() {
  // Initialize mvVISLAM
  vislam_ptr_ = mvVISLAM_Initialize
  (
    &mv_camera_config_,
    vislam_params_.readoutTime,
    vislam_params_.tbc,
    vislam_params_.ombc,
    vislam_params_.delta,
    vislam_params_.std0Tbc,
    vislam_params_.std0Ombc,
    vislam_params_.std0Delta,
    vislam_params_.accelMeasRange,
    vislam_params_.gyroMeasRange,
    vislam_params_.stdAccelMeasNoise,
    vislam_params_.stdGyroMeasNoise,
    vislam_params_.stdCamNoise,
    vislam_params_.minStdPixelNoise,
    vislam_params_.failHighPixelNoiseScaleFactor,
    vislam_params_.logDepthBootstrap,
    vislam_params_.useLogCameraHeight,
    vislam_params_.logCameraHeightBootstrap,
    vislam_params_.noInitWhenMoving,
    vislam_params_.limitedIMUbWtrigger,
    vislam_params_.staticMaskFilename.c_str(),
    vislam_params_.gpsImuTimeAlignment,
    &vislam_params_.tba[0],
    true
  );
  if (vislam_ptr_ == nullptr) {
    ROS_ERROR_STREAM("Could not initialize mvVISLAM object.");
    return false;
  }

  initialized_ = true;
  return true;
}

void SnapVio::PublishMapPoints(std::vector<mvVISLAMMapPoint>& vio_points,
  int64_t vio_frame_id, ros::Time image_timestamp) {

  snap_msgs::MapPointArray points_msg;
  points_msg.map_points.reserve(vio_points.size());
  points_msg.header.frame_id = "imu_start";
  points_msg.header.stamp = image_timestamp;
  points_msg.header.seq = vio_frame_id;

  sensor_msgs::PointCloud cloud_msg;
  cloud_msg.points.reserve(vio_points.size());
  cloud_msg.header.frame_id = "imu_start";
  cloud_msg.header.stamp = image_timestamp;
  cloud_msg.header.seq = vio_frame_id;

  sensor_msgs::ChannelFloat32 id_channel;
  id_channel.name = "id";
  id_channel.values.reserve(vio_points.size());

  sensor_msgs::ChannelFloat32 quality_channel;
  quality_channel.name = "point_quality";
  quality_channel.values.reserve(vio_points.size());

  for (auto it = vio_points.cbegin(); it != vio_points.cend(); ++it) {

    snap_msgs::MapPoint point;
    point.id = it->id;
    point.pixel_location[0] =  it->pixLoc[0];
    point.pixel_location[1] =  it->pixLoc[1];
    point.point.x = it->tsf[0];
    point.point.y = it->tsf[1];
    point.point.z = it->tsf[2];
    std::copy(&it->p_tsf[0][0], &it->p_tsf[0][0]+9, point.point_covariance.begin());
    point.depth = it->depth;
    point.depth_std_dev = it->depthErrorStdDev;
    point.point_quality = it->pointQuality;
    points_msg.map_points.push_back(point);

    geometry_msgs::Point32 cloud_pt;
    cloud_pt.x = it->tsf[0];
    cloud_pt.y = it->tsf[1];
    cloud_pt.z = it->tsf[2];
    cloud_msg.points.push_back(cloud_pt);

    id_channel.values.push_back(it->id);
    quality_channel.values.push_back(it->pointQuality);
  }

  cloud_msg.channels.push_back(id_channel);
  cloud_msg.channels.push_back(quality_channel);

  vio_points_publisher_.publish(points_msg);
  vio_cloud_publisher_.publish(cloud_msg);
}

void SnapVio::PublishVioData(mvVISLAMPose& vio_pose, int64_t vio_frame_id,
                             ros::Time vio_timestamp) {
  std::vector<geometry_msgs::TransformStamped> transforms;

  if(!snav_mode_)
  {
    geometry_msgs::TransformStamped odom_to_grav;
    odom_to_grav.transform.translation.x = 0;
    odom_to_grav.transform.translation.y = 0;
    odom_to_grav.transform.translation.z = 0;
    tf2::Quaternion q_imu( tf2::Vector3(0.0, 1.0, 0.0), 3.14159);
    tf2::convert(q_imu, odom_to_grav.transform.rotation);
    odom_to_grav.child_frame_id = "grav";
    odom_to_grav.header.frame_id = "odom";
    odom_to_grav.header.stamp = vio_timestamp;
    transforms.push_back(odom_to_grav);
  }

  geometry_msgs::TransformStamped grav_to_imu_start;
  tf2::Vector3 grav(vio_pose.gravity[0],
                    vio_pose.gravity[1],
                    vio_pose.gravity[2]);
  tf2::Vector3 unit_z(0,0,1);
  tf2::Quaternion q_grav( grav.cross(unit_z), grav.angle(unit_z));
  if(snav_mode_)
  {
    tf2::convert(q_grav.inverse(),grav_to_imu_start.transform.rotation);
    grav_to_imu_start.child_frame_id = "grav";
    grav_to_imu_start.header.frame_id = "imu_start";
  }
  else
  {
    tf2::convert(q_grav,grav_to_imu_start.transform.rotation);
    grav_to_imu_start.child_frame_id = "imu_start";
    grav_to_imu_start.header.frame_id = "grav";
  }
  grav_to_imu_start.header.stamp = vio_timestamp;
  transforms.push_back(grav_to_imu_start);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "imu_start";
  pose_msg.header.stamp = vio_timestamp;
  pose_msg.header.seq = vio_frame_id;
  // translate VIO pose to ROS pose
  tf2::Matrix3x3 R(
    vio_pose.bodyPose.matrix[0][0],
    vio_pose.bodyPose.matrix[0][1],
    vio_pose.bodyPose.matrix[0][2],
    vio_pose.bodyPose.matrix[1][0],
    vio_pose.bodyPose.matrix[1][1],
    vio_pose.bodyPose.matrix[1][2],
    vio_pose.bodyPose.matrix[2][0],
    vio_pose.bodyPose.matrix[2][1],
    vio_pose.bodyPose.matrix[2][2]);
  tf2::Quaternion q;
  R.getRotation(q);
  pose_msg.pose.position.x = vio_pose.bodyPose.matrix[0][3];
  pose_msg.pose.position.y = vio_pose.bodyPose.matrix[1][3];
  pose_msg.pose.position.z = vio_pose.bodyPose.matrix[2][3];
  pose_msg.pose.orientation.x = q.getX();
  pose_msg.pose.orientation.y = q.getY();
  pose_msg.pose.orientation.z = q.getZ();
  pose_msg.pose.orientation.w = q.getW();
  vio_pose_publisher_.publish(pose_msg);

  geometry_msgs::TransformStamped imu_start_to_imu;
  imu_start_to_imu.transform.translation.x = pose_msg.pose.position.x;
  imu_start_to_imu.transform.translation.y = pose_msg.pose.position.y;
  imu_start_to_imu.transform.translation.z = pose_msg.pose.position.z;
  imu_start_to_imu.transform.rotation.x = pose_msg.pose.orientation.x;
  imu_start_to_imu.transform.rotation.y = pose_msg.pose.orientation.y;
  imu_start_to_imu.transform.rotation.z = pose_msg.pose.orientation.z;
  imu_start_to_imu.transform.rotation.w = pose_msg.pose.orientation.w;
  imu_start_to_imu.child_frame_id = "imu";
  imu_start_to_imu.header.frame_id = "imu_start";
  imu_start_to_imu.header.stamp = vio_timestamp;

  if(snav_mode_)
  {
    tf2::Transform vio_tf, vio_tf_inv;
    tf2::convert(imu_start_to_imu.transform, vio_tf);
    vio_tf_inv = vio_tf.inverse();
    geometry_msgs::TransformStamped imu_to_imu_start;
    tf2::convert(vio_tf_inv, imu_to_imu_start.transform);
    imu_to_imu_start.child_frame_id = "imu_start";
    imu_to_imu_start.header.frame_id = "imu";
    imu_to_imu_start.header.stamp = vio_timestamp;
    transforms.push_back(imu_to_imu_start);
  }
  else
  {
    transforms.push_back(imu_start_to_imu);
  }

  // If running in snav_mode, reverse some transforms:
  // We assume that snav_ros will be responsible to /odom->/base_link
  // so we don't want to conflict with the tree and give /imu 2 parents

  // With snav_mode_==false, we publish:
  // odom->grav->imu_start->imu
  // With snav_mode_==true, we publish:
  // imu->imu_start->grav and snav_ros publishes odom->base_link,
  // with qflight_descriptions publshing base_link->imu

  //Send all the transforms
  tf_broadcaster_.sendTransform(transforms);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = vio_timestamp;
  odom_msg.header.frame_id = "imu_start";
  odom_msg.child_frame_id = "imu";
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.twist.twist.linear.x = vio_pose.velocity[0];
  odom_msg.twist.twist.linear.y = vio_pose.velocity[1];
  odom_msg.twist.twist.linear.z = vio_pose.velocity[2];
  odom_msg.twist.twist.angular.x = vio_pose.angularVelocity[0];
  odom_msg.twist.twist.angular.y = vio_pose.angularVelocity[1];
  odom_msg.twist.twist.angular.z = vio_pose.angularVelocity[2];
  //set the error covariance for the pose.
  //initialize twist covariance to zeros.
  for( int16_t i = 0; i < 6; i++ ) {
    for( int16_t j = 0; j < 6; j++ ) {
      odom_msg.pose.covariance[ i*6 + j ] = vio_pose.errCovPose[i][j];
      odom_msg.twist.covariance[ i*6 + j ] = 0.0;
    }
  }
  //set the error covariance for the velocity.
  for( int16_t i = 0; i < 3; i++ ) {
    for( int16_t j = 0; j < 3; j++ ) {
      odom_msg.twist.covariance[ i*6 + j ] = vio_pose.errCovVelocity[i][j];
    }
  }
  vio_odom_publisher_.publish(odom_msg);

  // Internal States
  snap_msgs::InternalStates states_msg;
  states_msg.header.frame_id = "imu_start";
  states_msg.header.stamp = vio_timestamp;
  states_msg.header.seq = vio_frame_id;
  states_msg.snav_mode = snav_mode_;
  states_msg.pose_quality = vio_pose.poseQuality;

  // gravity_camera_pose
  tf2::Matrix3x3 Rgcp(
    vio_pose.gravityCameraPose.matrix[0][0],
    vio_pose.gravityCameraPose.matrix[0][1],
    vio_pose.gravityCameraPose.matrix[0][2],
    vio_pose.gravityCameraPose.matrix[1][0],
    vio_pose.gravityCameraPose.matrix[1][1],
    vio_pose.gravityCameraPose.matrix[1][2],
    vio_pose.gravityCameraPose.matrix[2][0],
    vio_pose.gravityCameraPose.matrix[2][1],
    vio_pose.gravityCameraPose.matrix[2][2]);
  tf2::Quaternion qgcp;
  Rgcp.getRotation(qgcp);
  states_msg.gravity_camera_pose.translation.x = vio_pose.gravityCameraPose.matrix[0][3];
  states_msg.gravity_camera_pose.translation.y = vio_pose.gravityCameraPose.matrix[1][3];
  states_msg.gravity_camera_pose.translation.z = vio_pose.gravityCameraPose.matrix[2][3];
  states_msg.gravity_camera_pose.rotation.x = qgcp.getX();
  states_msg.gravity_camera_pose.rotation.y = qgcp.getY();
  states_msg.gravity_camera_pose.rotation.z = qgcp.getZ();
  states_msg.gravity_camera_pose.rotation.w = qgcp.getW();

  // time_alignment
  states_msg.time_alignment.data = ros::Duration(vio_pose.timeAlignment);

  // gravity
  states_msg.gravity.x = vio_pose.gravity[0];
  states_msg.gravity.y = vio_pose.gravity[1];
  states_msg.gravity.z = vio_pose.gravity[2];

  // err_cov_gravity
  std::copy(&vio_pose.errCovGravity[0][0], &vio_pose.errCovGravity[0][0]+9,
	    states_msg.err_cov_gravity.begin());

  // gyro_bias
  states_msg.gyro_bias.x = vio_pose.wBias[0];
  states_msg.gyro_bias.y = vio_pose.wBias[1];
  states_msg.gyro_bias.z = vio_pose.wBias[2];

  // accel_bias
  states_msg.accel_bias.x = vio_pose.aBias[0];
  states_msg.accel_bias.y = vio_pose.aBias[1];
  states_msg.accel_bias.z = vio_pose.aBias[2];

  // R_gyro_body
  tf2::Matrix3x3 Rgb(
    vio_pose.Rbg[0][0], vio_pose.Rbg[0][1], vio_pose.Rbg[0][2],
    vio_pose.Rbg[1][0], vio_pose.Rbg[1][1], vio_pose.Rbg[1][2],
    vio_pose.Rbg[2][0], vio_pose.Rbg[2][1], vio_pose.Rbg[2][2]);
  tf2::Quaternion qgb;
  Rgb.getRotation(qgb);
  states_msg.R_gyro_body.x = qgb.getX();
  states_msg.R_gyro_body.y = qgb.getY();
  states_msg.R_gyro_body.z = qgb.getZ();
  states_msg.R_gyro_body.w = qgb.getW();

  // a_accel_inv, a_gyro_inv
  std::copy(&vio_pose.aAccInv[0][0], &vio_pose.aAccInv[0][0]+9, states_msg.a_accel_inv.begin());
  std::copy(&vio_pose.aGyrInv[0][0], &vio_pose.aGyrInv[0][0]+9, states_msg.a_gyro_inv.begin());

  // tf_imu_camera
  tf2::Matrix3x3 Rbc(
    vio_pose.Rbc[0][0], vio_pose.Rbc[0][1], vio_pose.Rbc[0][2],
    vio_pose.Rbc[1][0], vio_pose.Rbc[1][1], vio_pose.Rbc[1][2],
    vio_pose.Rbc[2][0], vio_pose.Rbc[2][1], vio_pose.Rbc[2][2]);
  tf2::Quaternion qbc;
  Rbc.getRotation(qbc);
  states_msg.tf_imu_camera.translation.x = vio_pose.tbc[0];
  states_msg.tf_imu_camera.translation.y = vio_pose.tbc[1];
  states_msg.tf_imu_camera.translation.z = vio_pose.tbc[2];
  states_msg.tf_imu_camera.rotation.x = qbc.getX();
  states_msg.tf_imu_camera.rotation.y = qbc.getY();
  states_msg.tf_imu_camera.rotation.z = qbc.getZ();
  states_msg.tf_imu_camera.rotation.w = qbc.getW();

  // error_code
  states_msg.error_code = vio_pose.errorCode;

  vio_states_publisher_.publish(states_msg);


  // Update internal states
  latest_time_alignment_ = ros::Duration(vio_pose.timeAlignment);

  if (vio_pose.errorCode > 0) {
    ROS_ERROR_STREAM("[VISLAM] ERROR CODE = " << vio_pose.errorCode << ", Stamp: " << vio_timestamp );
  }
}
