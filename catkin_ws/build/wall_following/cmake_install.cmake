# Install script for directory: /mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/src/wall_following

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wall_following/srv" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/src/wall_following/srv/FindWall.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wall_following/cmake" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_following-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/include/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/share/roseus/ros/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/share/common-lisp/ros/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/share/gennodejs/ros/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/lib/python3/dist-packages/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/devel/lib/python3/dist-packages/wall_following")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_following.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wall_following/cmake" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_following-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wall_following/cmake" TYPE FILE FILES
    "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_followingConfig.cmake"
    "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_followingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wall_following" TYPE FILE FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/src/wall_following/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/wall_following" TYPE PROGRAM FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/find_wall.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/wall_following" TYPE PROGRAM FILES "/mnt/c/Users/chuchu/Downloads/ros-basics-in-5-days-real-robo--2025-07-31--3d780ecbc819/catkin_ws/build/wall_following/catkin_generated/installspace/wall_follower.py")
endif()

