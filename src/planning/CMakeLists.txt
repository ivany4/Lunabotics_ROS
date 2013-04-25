
rosbuild_add_library(planning
	a_star_node.cpp
	a_star_graph.cpp
	bezier_smooth.cpp
)
target_link_libraries(planning geometry)
