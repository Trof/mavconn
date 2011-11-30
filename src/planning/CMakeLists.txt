INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

PIXHAWK_EXECUTABLE(mavconn-missionplanner mavconn-missionplanner.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-missionplanner
  ${CXCORE_LIBRARY}
  mavconn_lcm
  lcm
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

PIXHAWK_EXECUTABLE(mavconn-missionplanner-new mavconn-missionplanner-new.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-missionplanner-new
  ${CXCORE_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  mavconn_lcm
  lcm
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

PIXHAWK_EXECUTABLE(mavconn-simulator mavconn-simulator.cc)
PIXHAWK_LINK_LIBRARIES(mavconn-simulator
  ${CXCORE_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  mavconn_lcm
  lcm
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)