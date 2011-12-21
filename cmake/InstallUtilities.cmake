##======================================================================
#
# PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
# Please see our website at <http://pixhawk.ethz.ch>
# 
#
# Original Authors:
#   @author Reto Grieder <www.orxonox.net>
#   @author Fabian Landau <www.orxonox.net>
# Contributing Authors (in alphabetical order):
#  
# Todo:
#
#
# (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
# 
# This file is part of the PIXHAWK project
# 
#     PIXHAWK is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     PIXHAWK is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.
# 
##========================================================================

FUNCTION(PIXHAWK_INSTALL)
  INSTALL(TARGETS ${ARGN}
    RUNTIME DESTINATION ${PIXHAWK_RUNTIME_INSTALL_PATH}
    LIBRARY DESTINATION ${PIXHAWK_LIBRARY_INSTALL_PATH}
    #ARCHIVE DESTINATION ${PIXHAWK_ARCHIVE_INSTALL_PATH}
  )
  SET(MAVCONN_HEADERS
	${CMAKE_SOURCE_DIR}/src/mavconn.h
	${CMAKE_BINARY_DIR}/src/MAVCONNConfig.h
  )
  INSTALL(FILES ${MAVCONN_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH})

  SET(MAVCONNNMEA_HEADERS
        ${CMAKE_SOURCE_DIR}/src/external/nmea/include/nmea/nmea.h
  )
  INSTALL(FILES ${MAVCONNNMEA_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH}/external/nmea/)

  SET(MAVCONNLCM_HEADERS
	${CMAKE_SOURCE_DIR}/src/comm/lcm/mavconn_mavlink_message_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/mavconn_mavlink_msg_container_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/camera_image_message_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/gl_overlay_message_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/obstacle_map_message_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/rgbd_camera_image_message_t.h
	${CMAKE_SOURCE_DIR}/src/comm/lcm/virtual_scan_message_t.h
  )
  INSTALL(FILES ${MAVCONNLCM_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH}/comm/lcm/)

  SET(MAVCONNIMG_HEADERS
	${CMAKE_SOURCE_DIR}/src/interface/shared_mem/PxSharedMemClient.h
	${CMAKE_SOURCE_DIR}/src/interface/shared_mem/PxSharedMemServer.h
	${CMAKE_SOURCE_DIR}/src/interface/shared_mem/PxSHM.h
	${CMAKE_SOURCE_DIR}/src/interface/shared_mem/PxSHMImageServer.h
	${CMAKE_SOURCE_DIR}/src/interface/shared_mem/PxSHMImageClient.h
  )
  INSTALL(FILES ${MAVCONNIMG_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH}/interface/shared_mem/)

  SET(MAVCONN_CORE_HEADERS
	${CMAKE_SOURCE_DIR}/src/core/MAVConnParamClient.h
	${CMAKE_SOURCE_DIR}/src/core/ParamClientCallbacks.h
  )
  INSTALL(FILES ${MAVCONN_CORE_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH}/core/)

  SET(MAVCONN_CORE_TIMER_HEADERS
	${CMAKE_SOURCE_DIR}/src/core/timer/OgreTimer.h
	${CMAKE_SOURCE_DIR}/src/core/timer/OgreTimerImp.GLX.h
	${CMAKE_SOURCE_DIR}/src/core/timer/Timer.h
  )
  INSTALL(FILES ${MAVCONN_CORE_TIMER_HEADERS} DESTINATION ${PIXHAWK_INCLUDE_INSTALL_PATH}/core/timer/)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  SET(MAVCONN_CAMERA_LIBRARIES
	${CMAKE_BINARY_DIR}/lib/libmvBlueFOX.so
	${CMAKE_BINARY_DIR}/lib/libmvBlueFOX.so.1.12.40
	${CMAKE_BINARY_DIR}/lib/libmvDeviceManager.so
	${CMAKE_BINARY_DIR}/lib/libmvDeviceManager.so.1.12.40
	${CMAKE_BINARY_DIR}/lib/libmvPropHandling.so
	${CMAKE_BINARY_DIR}/lib/libmvPropHandling.so.1.12.40
  )
  INSTALL(FILES ${MAVCONN_CAMERA_LIBRARIES} DESTINATION "/usr/local/lib")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

ENDFUNCTION(PIXHAWK_INSTALL)
