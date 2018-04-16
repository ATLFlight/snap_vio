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
cmake_minimum_required(VERSION 2.8.3)
project(snap_vio)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  snap_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  nodelet
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    image_transport
    snap_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    nodelet
  LIBRARIES snap_vio_nodelet
)

###########
## Build ##
###########

include_directories(include)
include_directories(src)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/mv
)

add_library(snap_vio src/snap_vio.cpp)
add_dependencies(snap_vio ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_vio
  ${catkin_LIBRARIES}
  mv1
)

add_executable(snap_vio_node src/snap_vio_node.cpp)
add_dependencies(snap_vio_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_vio_node
  ${catkin_LIBRARIES}
  snap_vio
)

add_library(snap_vio_nodelet src/snap_vio_nodelet.cpp)
add_dependencies(snap_vio_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(snap_vio_nodelet
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  snap_vio
)

include(CheckLibraryExists)
check_library_exists(snav_arm sn_send_vio_data "/usr/lib/libsnam_arm.so" HAVE_SNAV)
if( HAVE_SNAV)
  message("Found libsnav_arm; building snap_vio snav input node")

  if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
    add_definitions( -DQC_SOC_TARGET_APQ8096 )
    message("Building for APQ8096")
  else()
    add_definitions( -DQC_SOC_TARGET_APQ8074 )
    message("Building for APQ8074")
  endif()

  include_directories(snav)

  add_library(snav_vio_injector src/snav_vio_injector.cpp)
  add_dependencies(snav_vio_injector ${catkin_EXPORTED_TARGETS})
  target_link_libraries(snav_vio_injector
    ${catkin_LIBRARIES}
    snav_arm
  )

  add_executable(snav_vio_injector_node src/snav_vio_injector_node.cpp)
  add_dependencies(snav_vio_injector_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(snav_vio_injector_node
    ${catkin_LIBRARIES}
    snav_vio_injector
  )

  install(TARGETS snav_vio_injector_node snav_vio_injector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  check_library_exists(snav_arm sn_update_data "/usr/lib/libsnam_arm.so" HAVE_OLD_SNAV)
  if( HAVE_OLD_SNAV)
    message("Found libsnav_arm, but it is too old to use snav_vio_injector.  SNAV 1.2.59 or greater is required")
  else()
    message("Not found: libsnav_arm; Not building snap_vio snav input node")
  endif()
endif()



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  snap_vio snap_vio_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES snap_vio_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
