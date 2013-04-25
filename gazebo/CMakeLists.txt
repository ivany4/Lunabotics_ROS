cmake_minimum_required(VERSION 2.8)


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo>=1.6 REQUIRED)
  if (!GAZEBO_FOUND)
	message(FATAL_ERROR "Gazebo could not be located")
  else()
	message(STATUS "Gazebo successfully located")
  endif()
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

message(STATUS "Including directories ${GAZEBO_INCLUDE_DIRS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
message(STATUS "Linking directories ${GAZEBO_LIBRARY_DIRS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
get_rospack_flags(tf)
get_rospack_flags(lunabotics)

message(STATUS "Including directories ${roscpp_INCLUDE_DIRS} ${lunabotics_INCLUDE_DIRS} ${tf_INCLUDE_DIRS}")
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${lunabotics_INCLUDE_DIRS})
include_directories(${tf_INCLUDE_DIRS})
message(STATUS "Linking directories ${roscpp_LIBRARY_DIRS} ${lunabotics_LIBRARY_DIRS} ${tf_LIBRARY_DIRS}")
link_directories(${roscpp_LIBRARY_DIRS})
link_directories(${lunabotics_LIBRARY_DIRS})
link_directories(${tf_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

#Clock server
add_library(ClockServerPlugin SHARED src/ClockServerPlugin.cpp)
set_target_properties(ClockServerPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ClockServerPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(ClockServerPlugin ${roscpp_LIBRARIES})
install (TARGETS ClockServerPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Transform server
add_library(TransformServerPlugin SHARED src/TransformServerPlugin.cpp)
set_target_properties(TransformServerPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
set_target_properties(TransformServerPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
target_link_libraries(TransformServerPlugin ${roscpp_LIBRARIES} ${tf_LIBRARIES})
install (TARGETS TransformServerPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#All wheel drive controller
add_library(AllWheelSteeringPlugin SHARED src/AllWheelSteeringPlugin.cpp ../src/control/PIDController.cpp)
set_target_properties(AllWheelSteeringPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER} ${lunabotics_CFLAGS_OTHER}")
set_target_properties(AllWheelSteeringPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER} ${lunabotics_LDFLAGS_OTHER}")
target_link_libraries(AllWheelSteeringPlugin ${roscpp_LIBRARIES} ${lunabotics_LIBRARIES})
install (TARGETS AllWheelSteeringPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Differential drive controller
add_library(LunaboticsDiffDrivePlugin SHARED src/LunaboticsDiffDrivePlugin.cpp)
set_target_properties(LunaboticsDiffDrivePlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaboticsDiffDrivePlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaboticsDiffDrivePlugin ${roscpp_LIBRARIES})
install (TARGETS LunaboticsDiffDrivePlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Odometry publisher
add_library(LunaboticsOdometryPlugin SHARED src/LunaboticsOdometryPlugin.cpp)
set_target_properties(LunaboticsOdometryPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER} ${tf_CFLAGS_OTHER}")
set_target_properties(LunaboticsOdometryPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER} ${tf_LDFLAGS_OTHER}")
target_link_libraries(LunaboticsOdometryPlugin ${roscpp_LIBRARIES} ${tf_LIBRARIES})
install (TARGETS LunaboticsOdometryPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Laser publisher
add_library(LunaboticsLidarPlugin SHARED src/LunaboticsLidarPlugin.cpp)
set_target_properties(LunaboticsLidarPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaboticsLidarPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaboticsLidarPlugin ${roscpp_LIBRARIES})
install (TARGETS LunaboticsLidarPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)


