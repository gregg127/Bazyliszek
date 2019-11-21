# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multiple_machines: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imultiple_machines:/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multiple_machines_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_custom_target(_multiple_machines_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "multiple_machines" "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" ""
)

get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_custom_target(_multiple_machines_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "multiple_machines" "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiple_machines
)
_generate_msg_cpp(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiple_machines
)

### Generating Services

### Generating Module File
_generate_module_cpp(multiple_machines
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiple_machines
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multiple_machines_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multiple_machines_generate_messages multiple_machines_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_cpp _multiple_machines_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_cpp _multiple_machines_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiple_machines_gencpp)
add_dependencies(multiple_machines_gencpp multiple_machines_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiple_machines_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiple_machines
)
_generate_msg_eus(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiple_machines
)

### Generating Services

### Generating Module File
_generate_module_eus(multiple_machines
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiple_machines
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(multiple_machines_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(multiple_machines_generate_messages multiple_machines_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_eus _multiple_machines_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_eus _multiple_machines_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiple_machines_geneus)
add_dependencies(multiple_machines_geneus multiple_machines_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiple_machines_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiple_machines
)
_generate_msg_lisp(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiple_machines
)

### Generating Services

### Generating Module File
_generate_module_lisp(multiple_machines
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiple_machines
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multiple_machines_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multiple_machines_generate_messages multiple_machines_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_lisp _multiple_machines_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_lisp _multiple_machines_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiple_machines_genlisp)
add_dependencies(multiple_machines_genlisp multiple_machines_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiple_machines_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiple_machines
)
_generate_msg_nodejs(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiple_machines
)

### Generating Services

### Generating Module File
_generate_module_nodejs(multiple_machines
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiple_machines
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(multiple_machines_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(multiple_machines_generate_messages multiple_machines_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_nodejs _multiple_machines_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_nodejs _multiple_machines_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiple_machines_gennodejs)
add_dependencies(multiple_machines_gennodejs multiple_machines_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiple_machines_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines
)
_generate_msg_py(multiple_machines
  "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines
)

### Generating Services

### Generating Module File
_generate_module_py(multiple_machines
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multiple_machines_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multiple_machines_generate_messages multiple_machines_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/ProtocolMessage.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_py _multiple_machines_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/grzesiek/ros-inz/tutorial-1/catkin_ws/src/multiple_machines/msg/MotorsPwm.msg" NAME_WE)
add_dependencies(multiple_machines_generate_messages_py _multiple_machines_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiple_machines_genpy)
add_dependencies(multiple_machines_genpy multiple_machines_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiple_machines_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiple_machines)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiple_machines
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(multiple_machines_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiple_machines)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiple_machines
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(multiple_machines_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiple_machines)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiple_machines
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(multiple_machines_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiple_machines)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiple_machines
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(multiple_machines_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiple_machines
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(multiple_machines_generate_messages_py std_msgs_generate_messages_py)
endif()
