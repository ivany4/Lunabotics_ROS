#ifndef _PLANNING_GRAPH_H_
#define _PLANNING_GRAPH_H_

#include "a_star_node.h"
#include "../types.h"

namespace lunabotics {
namespace planning
{
	struct node_indexed {
		lunabotics::planning::node node;
		int index;
	};
	
	typedef std::vector<node_indexed> node_indexed_arr;
	
	struct line {
		point_f p1;
		point_f p2;
	};
		
	
	class path
	{
	private:
		int width;
		int height;
		bool initialized;
		map_grid map;
		node_arr nodes;
		node_arr corner_nodes;
		node_indexed_arr obstacle_nodes;
		
		bool lineIntersectsNodeAt(line l, int x, int y);
		bool linesIntersect(line l1, line l2);
		bool in_set(node_list set, node node);
		double distance(node node1, node node2);
		node_arr reconstruct_path(node_list came_from, node current);
		bool isObstacleBetweenNodes(node node1, node node2);
		point_arr pointRepresentation(node_arr graph, float resolution);
		point_indexed_arr pointRepresentation(node_indexed_arr graph, float resolution);	
		node_arr removeStraightPathWaypoints(node_arr originalGraph);
	public:
		path();
		path(map_grid map, int width, int height, int start_x, int start_y, int goal_x, int goal_y);
		node_arr cornerNodes();
		node_arr allNodes();
		node_indexed_arr closestObstacleNodes();
		point_arr cornerPoints(float resolution);
		point_arr allPoints(float resolution);
		point_indexed_arr closestObstaclePoints(float resolution);
		bool is_initialized();
		int8_t mapAt(int x, int y);
	};
}
}

#endif
