cmake_minimum_required(VERSION 3.0.2)
project(tregor_kalibr)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs cv_bridge)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/preview_saver.py
  scripts/stereo_rectify_rtabmap_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(PROGRAMS
  scripts/lock_uvc.sh
  scripts/run_usb_cam0_locked.sh
  scripts/run_usb_cam_delayed.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(_sh lock_uvc.sh run_usb_cam0_locked.sh run_usb_cam_delayed.sh)
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_sh}
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
endforeach()
