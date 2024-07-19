# Install script for directory: /root/ws_offboard_control/src/px4_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/ws_offboard_control/install/px4_msgs")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/px4_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_c/px4_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/lib/python3.8/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_typesupport_fastrtps_c/px4_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:/root/ws_offboard_control/build/px4_msgs:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_typesupport_fastrtps_cpp/px4_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_typesupport_introspection_c/px4_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/root/ws_offboard_control/build/px4_msgs:/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_cpp/px4_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/px4_msgs" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_typesupport_introspection_cpp/px4_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/libpx4_msgs__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/root/ws_offboard_control/install/px4_msgs/lib/python3.8/site-packages/px4_msgs/__init__.py"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/msg" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/msg/" REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/srv" TYPE DIRECTORY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/srv/" REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so"
         OLD_RPATH "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs:/root/ws_offboard_control/build/px4_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/root/ws_offboard_control/build/px4_msgs/CMakeFiles/px4_msgs__rosidl_typesupport_fastrtps_c__pyext.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so"
         OLD_RPATH "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs:/root/ws_offboard_control/build/px4_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_introspection_c.cpython-38-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/root/ws_offboard_control/build/px4_msgs/CMakeFiles/px4_msgs__rosidl_typesupport_introspection_c__pyext.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so"
         OLD_RPATH "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs:/root/ws_offboard_control/build/px4_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/px4_msgs/px4_msgs_s__rosidl_typesupport_c.cpython-38-aarch64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/root/ws_offboard_control/build/px4_msgs/CMakeFiles/px4_msgs__rosidl_typesupport_c__pyext.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_generator_py/px4_msgs/libpx4_msgs__python.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so"
         OLD_RPATH "/root/ws_offboard_control/build/px4_msgs:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpx4_msgs__python.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActionRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorArmed.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorControlsStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorMotors.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorOutputs.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServos.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServosTrim.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorTest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AdcReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Airspeed.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedValidated.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedWind.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckReply.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AutotuneAttitudeControlStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/BatteryStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Buffer128.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ButtonEvent.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraCapture.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraTrigger.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CanInterfaceStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CellularStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CollisionConstraints.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CollisionReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ConfigOverrides.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ControlAllocatorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Cpuload.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanResponse.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugArray.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugKeyValue.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugValue.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugVect.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DifferentialDriveSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DifferentialPressure.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DistanceSensor.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ekf2Timestamps.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource1d.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource2d.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource3d.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias3d.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorEventFlags.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorGpsStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorInnovations.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSelectorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSensorBias.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStates.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatusFlags.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Event.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailsafeFlags.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailureDetectorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FigureEightStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FlightPhaseEstimation.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTarget.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetEstimator.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FuelTankStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeneratorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceResult.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalControls.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceAttitudeStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceInformation.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceSetAttitude.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerInformation.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetAttitude.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetManualControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GotoSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioConfig.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioIn.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioOut.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsDump.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsInjectData.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Gripper.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HealthReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HeaterStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HomePosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HoverThrustEstimate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InputRc.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InternalCombustionEngineStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IridiumsbdStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IrlockReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGear.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGearWheel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetInnovations.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetPose.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LaunchDetectionStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LedControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LogMessage.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LoggerStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagWorkerData.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagnetometerBiasEstimate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSwitches.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkLog.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkTunnel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatResponse.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Mission.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MissionResult.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ModeCompleted.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MountOrientation.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NavigatorMissionItem.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NormalizedUnsignedSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NpfgStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ObstacleDistance.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OffboardControlMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OnboardComputerStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestLarge.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestMedium.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbitStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterResetRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetUsedRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueResponse.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterUpdate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ping.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerLandingStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpointTriplet.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerButtonState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerMonitor.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PpsCapture.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PwmInput.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Px4ioStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellReq.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellRetval.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RadioStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RateCtrlStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcChannels.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcParameterMap.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentReply.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverAckermannGuidanceStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Rpm.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlTimeEstimate.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SatelliteInfo.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccelFifo.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAirflow.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorBaro.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCombined.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCorrection.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGnssRelative.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGps.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyro.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFft.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFifo.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorHygrometer.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorMag.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorOpticalFlow.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorPreflightMag.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorSelection.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorUwb.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatusImu.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SystemPower.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TakeoffStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TaskStackInfo.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TecsStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TelemetryStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TiltrotorExtraControls.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TimesyncStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectoryBezier.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectorySetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectoryWaypoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TransponderReport.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TuneControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterValue.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStream.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStreamAck.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UnregisterExtComponent.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAcceleration.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAirData.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularAccelerationSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularVelocity.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitude.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitudeSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommand.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommandAck.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleConstraints.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleControlMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleGlobalPosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImu.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImuStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLandDetected.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPosition.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPositionSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleMagnetometer.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOdometry.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlow.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlowVel.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRatesSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRoi.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleThrustSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleTorqueSetpoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleTrajectoryBezier.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleTrajectoryWaypoint.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VelocityLimits.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VtolVehicleStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/WheelEncoders.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Wind.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/msg/YawEstimatorStatus.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/srv" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_adapter/px4_msgs/srv/VehicleCommand.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActionRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorArmed.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorControlsStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorMotors.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorOutputs.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorServos.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorServosTrim.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ActuatorTest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/AdcReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Airspeed.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/AirspeedValidated.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/AirspeedWind.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ArmingCheckReply.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ArmingCheckRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/AutotuneAttitudeControlStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/BatteryStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Buffer128.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ButtonEvent.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CameraCapture.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CameraStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CameraTrigger.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CanInterfaceStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CellularStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CollisionConstraints.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/CollisionReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ConfigOverrides.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ControlAllocatorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Cpuload.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DatamanRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DatamanResponse.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DebugArray.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DebugKeyValue.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DebugValue.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DebugVect.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DifferentialDriveSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DifferentialPressure.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/DistanceSensor.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Ekf2Timestamps.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EscReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EscStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorAidSource1d.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorAidSource2d.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorAidSource3d.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorBias.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorBias3d.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorEventFlags.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorGpsStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorInnovations.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorSelectorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorSensorBias.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorStates.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/EstimatorStatusFlags.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Event.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FailsafeFlags.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FailureDetectorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FigureEightStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FlightPhaseEstimation.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FollowTarget.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FollowTargetEstimator.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FollowTargetStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/FuelTankStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GeneratorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GeofenceResult.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GeofenceStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalControls.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalDeviceAttitudeStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalDeviceInformation.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalDeviceSetAttitude.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalManagerInformation.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalManagerSetAttitude.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalManagerSetManualControl.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GimbalManagerStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GotoSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpioConfig.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpioIn.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpioOut.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpioRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpsDump.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/GpsInjectData.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Gripper.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/HealthReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/HeaterStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/HomePosition.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/HoverThrustEstimate.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/InputRc.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/InternalCombustionEngineStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/IridiumsbdStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/IrlockReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LandingGear.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LandingGearWheel.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LandingTargetInnovations.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LandingTargetPose.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LaunchDetectionStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LedControl.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LogMessage.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/LoggerStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MagWorkerData.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MagnetometerBiasEstimate.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ManualControlSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ManualControlSwitches.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MavlinkLog.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MavlinkTunnel.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MessageFormatRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MessageFormatResponse.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Mission.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MissionResult.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ModeCompleted.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/MountOrientation.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/NavigatorMissionItem.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/NormalizedUnsignedSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/NpfgStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ObstacleDistance.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OffboardControlMode.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OnboardComputerStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OrbTest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OrbTestLarge.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OrbTestMedium.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/OrbitStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ParameterResetRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ParameterSetUsedRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ParameterSetValueRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ParameterSetValueResponse.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/ParameterUpdate.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Ping.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PositionControllerLandingStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PositionControllerStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PositionSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PositionSetpointTriplet.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PowerButtonState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PowerMonitor.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PpsCapture.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/PwmInput.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Px4ioStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/QshellReq.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/QshellRetval.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RadioStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RateCtrlStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RcChannels.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RcParameterMap.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RegisterExtComponentReply.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RegisterExtComponentRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RoverAckermannGuidanceStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Rpm.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RtlStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/RtlTimeEstimate.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SatelliteInfo.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorAccel.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorAccelFifo.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorAirflow.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorBaro.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorCombined.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorCorrection.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorGnssRelative.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorGps.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorGyro.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorGyroFft.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorGyroFifo.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorHygrometer.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorMag.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorOpticalFlow.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorPreflightMag.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorSelection.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorUwb.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorsStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SensorsStatusImu.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/SystemPower.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TakeoffStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TaskStackInfo.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TecsStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TelemetryStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TiltrotorExtraControls.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TimesyncStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TrajectoryBezier.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TrajectorySetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TrajectoryWaypoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TransponderReport.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/TuneControl.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/UavcanParameterRequest.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/UavcanParameterValue.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/UlogStream.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/UlogStreamAck.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/UnregisterExtComponent.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAcceleration.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAirData.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAngularAccelerationSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAngularVelocity.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAttitude.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleAttitudeSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleCommand.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleCommandAck.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleConstraints.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleControlMode.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleGlobalPosition.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleImu.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleImuStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleLandDetected.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleLocalPosition.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleLocalPositionSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleMagnetometer.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleOdometry.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleOpticalFlow.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleOpticalFlowVel.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleRatesSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleRoi.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleThrustSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleTorqueSetpoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleTrajectoryBezier.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VehicleTrajectoryWaypoint.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VelocityLimits.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/VtolVehicleStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/WheelEncoders.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/Wind.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/msg" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/msg/YawEstimatorStatus.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/srv" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/srv/VehicleCommand.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/srv" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_cmake/srv/VehicleCommand_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/srv" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_cmake/srv/VehicleCommand_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/px4_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/px4_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/environment" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/packages/px4_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cppExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cppExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cppExport.cmake"
         "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake/px4_msgs__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/CMakeFiles/Export/f93cf43fbe18b15a28cba0c39d379bae/px4_msgs__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES "/root/ws_offboard_control/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs/cmake" TYPE FILE FILES
    "/root/ws_offboard_control/build/px4_msgs/ament_cmake_core/px4_msgsConfig.cmake"
    "/root/ws_offboard_control/build/px4_msgs/ament_cmake_core/px4_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/px4_msgs" TYPE FILE FILES "/root/ws_offboard_control/src/px4_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/root/ws_offboard_control/build/px4_msgs/px4_msgs__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/root/ws_offboard_control/build/px4_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
