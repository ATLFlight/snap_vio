#!/usr/bin/env python
# /****************************************************************************
#  *   Copyright (c) 2018 Michael Shomin. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name ATLFlight nor the names of its contributors may be
#  *    used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#  ****************************************************************************/

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from snap_msgs.msg import MapPointArray

import cv2
from cv_bridge import CvBridge, CvBridgeError

image_sub = message_filters.Subscriber('/downward/image_raw', Image)
point_sub = message_filters.Subscriber('/downward/vio/map_points', MapPointArray)
overlay_pub = rospy.Publisher("/downward/image_overlay",Image, queue_size=5)
bridge = CvBridge()

def callback(image, map_points):
  cv_image = bridge.imgmsg_to_cv2(image, "mono8")
  rgb_cv_image = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2RGB)

  for pt in map_points.map_points:
    #High quality points - Green
    if pt.point_quality == 2:
      color = (0,255,0)
      r = 7
    #Low quality points - Red
    elif pt.point_quality == 1:
      color = (0,0,255)
      r = 5
    elif pt.point_quality == 0:
      color = (255,0,0)
      r = 4
    if pt.pixel_location[0] > 0 and pt.pixel_location[1] > 0:
      cv2.circle(rgb_cv_image, (int(pt.pixel_location[0]),
                                int(pt.pixel_location[1])), r, color)
  img = bridge.cv2_to_imgmsg(rgb_cv_image, "bgr8")
  img.header.stamp = rospy.get_rostime()
  img.header.frame_id = 'dfc'
  overlay_pub.publish(img)

rospy.init_node('overlay_test')

ts = message_filters.TimeSynchronizer([image_sub, point_sub], 10)
ts.registerCallback(callback)

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
