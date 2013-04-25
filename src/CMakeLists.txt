rosbuild_add_boost_directories()


MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

add_subdirectory(geometry)
add_subdirectory(planning)
add_subdirectory(control)

rosbuild_add_executable(luna_gui_gw luna_gui_gw.cpp)
rosbuild_add_executable(luna_gui_listener luna_gui_listener.cpp types.cpp)
rosbuild_add_executable(luna_mech_gw luna_mech_gw.cpp)
rosbuild_add_executable(luna_fear luna_fear.cpp)
rosbuild_add_executable(luna_driver luna_driver.cpp types.cpp)
rosbuild_add_executable(luna_planner luna_planner.cpp)
rosbuild_add_executable(luna_slam luna_slam.cpp)
rosbuild_add_executable(luna_aut_gw luna_aut_gw.cpp)

target_link_libraries(luna_driver planning)
target_link_libraries(luna_gui_gw geometry)
rosbuild_link_boost(luna_gui_listener system signals)
rosbuild_link_boost(luna_gui_gw system)


#building protocol buffers
include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

rosbuild_add_library(protos
	../protos_gen/Point.pb.cc 
	../protos_gen/Twist.pb.cc 
	../protos_gen/SteeringModeType.pb.cc 
	../protos_gen/Telemetry.pb.cc 
	../protos_gen/Telecommand.pb.cc
	../protos_gen/AllWheelState.pb.cc
	../protos_gen/AllWheelControl.pb.cc
)
target_link_libraries(protos ${PROTOBUF_LIBRARY})

target_link_libraries(luna_gui_listener protos)
target_link_libraries(luna_gui_gw protos)

HEADER_DIRECTORIES(DIRS_TO_INCLUDE)
message (STATUS "Include directories will be ${DIRS_TO_INCLUDE}")
include_directories(${DIRS_TO_INCLUDE})
