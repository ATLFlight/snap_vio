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
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>

#include "snap_vio/snap_vio.hpp"

namespace snap_vio {
  class VioNodelet : public nodelet::Nodelet {
  public:
    VioNodelet() : running_(false) {}
    ~VioNodelet();

  private:
    virtual void onInit();
    virtual void devicePoll();
    volatile bool running_;
    boost::mutex mutex;
    boost::shared_ptr<SnapVio> driver_;
//    boost::shared_ptr<boost::thread> deviceThread_;
  };

  VioNodelet::~VioNodelet() {
    if (running_) {
      mutex.lock();
      driver_->Stop();
      mutex.unlock();
    }
  }

  void VioNodelet::onInit() {
    ros::NodeHandle nh(getMTNodeHandle());
    ros::NodeHandle pnh(getMTPrivateNodeHandle());

    driver_.reset(new SnapVio(nh, pnh));

    if (driver_->Start()) {
      running_ = true;
//      deviceThread_ = boost::shared_ptr< boost::thread >
//	(new boost::thread(boost::bind(&VioNodelet::devicePoll, this)));
    } else {
      NODELET_ERROR("Unable to start VIO.");
      driver_.reset();
    }
  }

  void VioNodelet::devicePoll() {
//    while(running_){
//      mutex.lock();
//      //driver_->DoSomething();
//      mutex.unlock();
//    }
  }


};

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(snap_vio, driver,snap_vio::VioNodelet, nodelet::Nodelet);
