# Install script for directory: /home/dassdinho/catkin_ws_8/src/jackal_velodyne_vlp16_lidar_ros_nav_stack

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dassdinho/catkin_ws_8/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dassdinho/catkin_ws_8/build/jackal_velodyne_vlp16_lidar_ros_nav_stack/catkin_generated/installspace/jackal_velodyne_vlp16_lidar_ros_nav_stack.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_velodyne_vlp16_lidar_ros_nav_stack/cmake" TYPE FILE FILES
    "/home/dassdinho/catkin_ws_8/build/jackal_velodyne_vlp16_lidar_ros_nav_stack/catkin_generated/installspace/jackal_velodyne_vlp16_lidar_ros_nav_stackConfig.cmake"
    "/home/dassdinho/catkin_ws_8/build/jackal_velodyne_vlp16_lidar_ros_nav_stack/catkin_generated/installspace/jackal_velodyne_vlp16_lidar_ros_nav_stackConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_velodyne_vlp16_lidar_ros_nav_stack" TYPE FILE FILES "/home/dassdinho/catkin_ws_8/src/jackal_velodyne_vlp16_lidar_ros_nav_stack/package.xml")
endif()

