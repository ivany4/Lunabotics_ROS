#include "a_star_node.h"

class a_star_graph
{
private:
	int width;
	int height;
	bool in_set(std::list<a_star_node> set, a_star_node node);
	double distance(a_star_node node1, a_star_node node2);
	std::vector<a_star_node> reconstruct_path(std::list<a_star_node> came_from, a_star_node current);
	bool isObstacleBetweenNodes(a_star_node node1, a_star_node node2, std::vector<int8_t> map, int width);
public:
	a_star_graph();
	std::vector<a_star_node> find_path(std::vector<int8_t> map, 
								int width, int height, 
								int start_x, int start_y, 
								int goal_x, int goal_y);
	std::vector<a_star_node> removeIntermediateWaypoints(std::vector<a_star_node> originalGraph);
	std::vector<a_star_node> removeIntermediateWaypoints(std::vector<a_star_node> originalGraph, 
															std::vector<int8_t> map,
															int width);
	
};
