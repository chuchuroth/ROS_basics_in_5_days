# Install script for directory: /mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/src/scan_test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/scan_test/catkin_generated/installspace/scan_test.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scan_test/cmake" TYPE FILE FILES
    "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/scan_test/catkin_generated/installspace/scan_testConfig.cmake"
    "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/scan_test/catkin_generated/installspace/scan_testConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scan_test" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/src/scan_test/package.xml")
endif()

