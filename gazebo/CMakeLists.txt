cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

#Basic cube controller
add_library(LunaSimpleControlPlugin SHARED src/ros_model_plugin.cc)
set_target_properties(LunaSimpleControlPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaSimpleControlPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaSimpleControlPlugin ${roscpp_LIBRARIES})
install (TARGETS LunaSimpleControlPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Differential drive controller
add_library(LunaDiffDrivePlugin SHARED src/MotorControlPlugin.cc)
set_target_properties(LunaDiffDrivePlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaDiffDrivePlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaDiffDrivePlugin ${roscpp_LIBRARIES})
install (TARGETS LunaDiffDrivePlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Odometry publisher
add_library(LunaOdometryPlugin SHARED src/luna_gazebo_odometry_plugin.cpp)
set_target_properties(LunaOdometryPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaOdometryPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaOdometryPlugin ${roscpp_LIBRARIES})
install (TARGETS LunaOdometryPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)
