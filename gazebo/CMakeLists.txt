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

#After installation of 1.6 over 1.5 it got messed up. So, forcing a newer version
set(GAZEBO_INCLUDE_DIRS "/usr/include/gazebo-1.6;/usr/include/gazebo-1.6/gazebo")
set(GAZEBO_LIBRARY_DIRS "/usr/include/gazebo-1.6/plugins")

message(STATUS "Including directories ${GAZEBO_INCLUDE_DIRS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
message(STATUS "Linking directories ${GAZEBO_LIBRARY_DIRS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
message(STATUS "Including directories ${roscpp_INCLUDE_DIRS}")
include_directories(${roscpp_INCLUDE_DIRS})
message(STATUS "Linking directories ${roscpp_LIBRARY_DIRS}")
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

#Clock server
add_library(ClockServerPlugin SHARED src/ClockServerPlugin.cpp)
set_target_properties(ClockServerPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ClockServerPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(ClockServerPlugin ${roscpp_LIBRARIES})
install (TARGETS ClockServerPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#All wheel drive controller
add_library(AllWheelSteeringPlugin SHARED src/AllWheelSteeringPlugin.cpp)
set_target_properties(AllWheelSteeringPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(AllWheelSteeringPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(AllWheelSteeringPlugin ${roscpp_LIBRARIES})
install (TARGETS AllWheelSteeringPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Differential drive controller
add_library(LunaboticsDiffDrivePlugin SHARED src/LunaboticsDiffDrivePlugin.cpp)
set_target_properties(LunaboticsDiffDrivePlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaboticsDiffDrivePlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaboticsDiffDrivePlugin ${roscpp_LIBRARIES})
install (TARGETS LunaboticsDiffDrivePlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

#Odometry publisher
add_library(LunaboticsOdometryPlugin SHARED src/LunaboticsOdometryPlugin.cpp)
set_target_properties(LunaboticsOdometryPlugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(LunaboticsOdometryPlugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(LunaboticsOdometryPlugin ${roscpp_LIBRARIES})
install (TARGETS LunaboticsOdometryPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)


