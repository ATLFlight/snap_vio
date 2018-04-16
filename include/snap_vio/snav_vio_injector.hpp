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
#ifndef _SNAV_VIO_INJECTOR_H_
#define _SNAV_VIO_INJECTOR_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <snap_msgs/InternalStates.h>
#include <snap_msgs/MapPointArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <snav/snapdragon_navigator.h>

class SnavInterface
{
public:
  /**
   * Constructor.
   * @param nh
   *   nodehandle to intialize the node.
   * @param pnh
   *   private namespace nodehandle for this node
   */
  SnavInterface(ros::NodeHandle nh, ros::NodeHandle pnh);

  ~SnavInterface();

  void VioCallback(const nav_msgs::OdometryConstPtr& odom,
                   const snap_msgs::InternalStatesConstPtr& states,
                   const snap_msgs::MapPointArrayConstPtr& points);

private:
  void GetDSPTimeOffset();

  message_filters::Subscriber<nav_msgs::Odometry>* vio_odom_subscriber_;
  message_filters::Subscriber<snap_msgs::InternalStates>* vio_states_subscriber_;
  message_filters::Subscriber<snap_msgs::MapPointArray>* vio_points_subscriber_;
  message_filters::TimeSynchronizer<nav_msgs::Odometry,
                                    snap_msgs::InternalStates,
                                    snap_msgs::MapPointArray>* vio_sync_;

  //public namespace nodehandle
  ros::NodeHandle nh_;
  //private namespace nodehandle
  ros::NodeHandle pnh_;

  SnavCachedData *cached_data_;
  ros::Time last_sn_update_;

  bool simulation_;
  int64_t dsp_offset_in_ns_;
};

#endif
