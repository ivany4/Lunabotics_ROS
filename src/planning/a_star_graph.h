#ifndef _PLANNING_GRAPH_H_
#define _PLANNING_GRAPH_H_

#include "a_star_node.h"
#include "../types.h"

namespace lunabotics {
	
struct IndexedNode {
	Node node;
	int index;
};

typedef std::vector<IndexedNode> IndexedNodeArr;


class Path {
private:
	MapData map;
	bool initialized;
	bool use_cspace;
	NodeArr nodes;
	NodeArr corner_nodes;
	IndexedNodeArr obstacle_nodes;
	Rect robot_dimensions;
	
	bool lineIntersectsNodeAt(Line line, int x, int y);
	bool linesIntersect(Line line1, Line line2);
	bool in_set(NodeList set, Node node);
	bool in_set(NodeList set, Node node, Node &original);
	double distance(Node node1, Node node2);
	NodeArr reconstruct_path(NodeList came_from, Node current);
	bool isObstacleBetweenNodes(Node node1, Node node2);
	bool isEssentialNodeInBetween(int node1_idx, int node2_idx, NodeArr arr);
	PointArr pointRepresentation(NodeArr graph, float resolution);
	IndexedPointArr pointRepresentation(IndexedNodeArr graph, float resolution);	
	NodeArr removeStraightPathWaypoints(NodeArr originalGraph);
public:
	Path();
	Path(MapData map, Point start);
	Path(MapData map, Point start, Rect robotDimensions);
	Path(MapData map, Point start, Rect robotDimensions, bool useCSpace);
	NodeArr cornerNodes();
	NodeArr allNodes();
	void appendGoal(Point goal);
	IndexedNodeArr closestObstacleNodes();
	PointArr cornerPoints(float resolution);
	PointArr allPoints(float resolution);
	IndexedPointArr closestObstaclePoints(float resolution);
	bool is_initialized();
	bool obstacles_in_circle(Node center, int radius, double &closestDistance);
};

typedef Path * PathPtr;
void PrintNodes(NodeArr arr, std::string label);
}

#endif
