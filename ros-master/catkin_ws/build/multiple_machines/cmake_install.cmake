# Install script for directory: /home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multiple_machines/msg" TYPE FILE FILES
    "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
    "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multiple_machines/cmake" TYPE FILE FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/build/multiple_machines/catkin_generated/installspace/multiple_machines-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/include/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/share/roseus/ros/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/share/common-lisp/ros/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/share/gennodejs/ros/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/lib/python2.7/dist-packages/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/devel/lib/python2.7/dist-packages/multiple_machines")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/build/multiple_machines/catkin_generated/installspace/multiple_machines.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multiple_machines/cmake" TYPE FILE FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/build/multiple_machines/catkin_generated/installspace/multiple_machines-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multiple_machines/cmake" TYPE FILE FILES
    "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/build/multiple_machines/catkin_generated/installspace/multiple_machinesConfig.cmake"
    "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/build/multiple_machines/catkin_generated/installspace/multiple_machinesConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multiple_machines" TYPE FILE FILES "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/package.xml")
endif()

