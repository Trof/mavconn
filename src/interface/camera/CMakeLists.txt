INCLUDE_DIRECTORIES(.
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GLIBMM2_MAIN_INCLUDE_DIR}
  ${GLIBMM2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${SIGC++_INCLUDE_DIR}
  ${DC_INCLUDE_DIR}
  ${DC_INCLUDE_DIR}/../
  ../../external/mvimpact
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # Linux specific code
    SET(LINUX_SYSTEM_FOUND true)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

LINK_DIRECTORIES(
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

PIXHAWK_EXECUTABLE(mavconn-imagecapture mavconn-imagecapture.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-imagecapture
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  mavconn_lcm
  mavconn_shm
  lcm
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

PIXHAWK_EXECUTABLE(mavconn-replay mavconn-replay.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-replay
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GLIBMM2_LIBRARY}
  mavconn_lcm
  mavconn_shm
  lcm
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

SET_SOURCE_FILES(CAMERA_SRC_FILES
  PxCamera.cc
  PxCameraCapture.cc
  PxCameraManager.cc
  PxCameraManagerFactory.cc
  PxFireflyCamera.cc
  PxFireflyCameraManager.cc
  PxFireflyStereoCamera.cc
#  PxOpenCVCamera.cc
#  PxOpenCVCameraManager.cc
  PxStereoCamera.cc
)

IF(LINUX_SYSTEM_FOUND)
SET_SOURCE_FILES(CAMERA_SRC_FILES
  ${CAMERA_SRC_FILES}
  PxBluefoxCamera.cc
  PxBluefoxCameraManager.cc
  PxBluefoxStereoCamera.cc
)
ENDIF(LINUX_SYSTEM_FOUND)

# WARNING: Do not link mvDeviceManager and OPENCV_HIGHGUI_LIBRARY
# to the same executable/library as there is some unknown cross-interaction
# between these 2 libraries which causes the camera API to work incorrectly.
PIXHAWK_LIBRARY(mavconn_cam SHARED ${CAMERA_SRC_FILES})
PIXHAWK_LINK_LIBRARIES(mavconn_cam
  ${DC_LIBRARY_OPTIMIZED}
  ${GLIBMM2_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
#  ${OPENCV_HIGHGUI_LIBRARY}
  ${SIGC++_LIBRARY}
  lcm
  mavconn_lcm
  mavconn_shm
)

IF(LINUX_SYSTEM_FOUND)
PIXHAWK_LINK_LIBRARIES(mavconn_cam
  mvDeviceManager
)
ENDIF(LINUX_SYSTEM_FOUND)

SET_TARGET_PROPERTIES(mavconn_cam PROPERTIES COMPILE_FLAGS "-D_REENTRANT")

PIXHAWK_EXECUTABLE(mavconn-camera
  mavconn-camera.cc
)
PIXHAWK_LINK_LIBRARIES(mavconn-camera
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
    ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  mavconn_cam
)

PIXHAWK_EXECUTABLE(mavconn-view mavconn-view.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-view
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  mavconn_lcm
  lcm
  mavconn_shm
)
