/* Check http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode for details */
#include "ros/ros.h"
#include "a_star_graph.h"
#include <math.h>
#include <list>
#include <sstream>
#include <algorithm>
#include "float.h"
using namespace lunabotics;

//---------------------------- CONSTRUCTOR / DESCTRUCTOR ------------------------//
Path::Path(): map(), initialized(false), nodes(), corner_nodes()
{
}

Path::Path(MapData map, Point start): map(map), initialized(true), use_cspace(false), nodes(), corner_nodes(), robot_dimensions()
{
	//ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start.x, start.y, goal.x, goal.y);	
	if (map.at(start) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); this->initialized = false; }
    
    this->nodes.push_back(Node(start));
}

Path::Path(MapData map, Point start, Rect robotDimensions): map(map), initialized(true), use_cspace(true), nodes(), corner_nodes(), robot_dimensions(robotDimensions)
{
	//ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start.x, start.y, goal.x, goal.y);	
	if (map.at(start) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); this->initialized = false; }
    
    this->nodes.push_back(Node(start));
}

void Path::appendGoal(Point goal)
{
    if (map.at(goal) > OCC_THRESHOLD) { ROS_ERROR("Goal cell is occupied"); this->initialized = false; }
    
    if (this->initialized) {
		
		NodeList open_set;
		NodeList closed_set;
		NodeList came_from;
		NodeArr graph;
		
	    
	    Node goal_node = Node(goal);  
	    goal_node.essential = true;
	    Node start_node = this->nodes.at(this->nodes.size()-1);
	    start_node.parent_x = start_node.x;
	    start_node.parent_y = start_node.y;
	    start_node.essential = true;
		
		
		
	    start_node.setH(this->distance(start_node, goal_node));	//Distance

	    open_set.push_back(start_node);
	    
	    std::stringstream sstr;
		sstr << start_node << " and " << goal_node;
		//ROS_INFO("Running A* between %s", sstr.str().c_str());
	    
	    while (!open_set.empty()) {
	        open_set.sort();
			Node current = open_set.front();
			sstr.str(std::string());
			sstr << current;
			//ROS_INFO("Node with smallest F=%.3f is %s", current.getF(), sstr.str().c_str());
	
	        if (current == goal_node) {
				//ROS_INFO("Found goal %s. Setting essential", sstr.str().c_str());
				current.essential = true;
				graph = this->reconstruct_path(came_from, current);
			//	ROS_INFO("Path reconstructed");
				reverse(graph.begin(), graph.end());
				break;
			}
			
	        open_set.remove(current);
	        closed_set.push_back(current);
	        
	        NodeList neighbours = this->use_cspace ? current.neighbours(this->map, this->robot_dimensions) : current.neighbours(this->map);
	        
	        for (NodeList::iterator it = neighbours.begin(); it != neighbours.end(); it++) {
				Node neighbour = *it;
				if (this->in_set(closed_set, neighbour)) {
					continue;
				}
				
				double tentative_G = current.getG() + this->distance(current, neighbour);
				
				if (!this->in_set(open_set, neighbour) || tentative_G <= neighbour.getG()) {
					came_from.push_back(current);
					neighbour.parent_x = current.x;
					neighbour.parent_y = current.y;
					neighbour.setG(tentative_G);
					neighbour.setH(this->distance(neighbour, goal_node));
					if (!this->in_set(open_set, neighbour)) {
						open_set.push_back(neighbour);
					}
				}
			}
	    }
	
	
		if (graph.size() > 1) {
			//ROS_INFO("Appending %d nodes to the path of %d", (int)graph.size(), (int)this->nodes.size());
			this->nodes.insert(this->nodes.end(), graph.begin()+1, graph.end());
		}
	}
}

//-------------------- GETTER / SETTER --------------------------//

PointArr Path::cornerPoints(float resolution)
{
	return this->pointRepresentation(this->cornerNodes(), resolution);
}

PointArr Path::allPoints(float resolution)
{
	return this->pointRepresentation(this->allNodes(), resolution);
}

IndexedPointArr Path::closestObstaclePoints(float resolution)
{
	return this->pointRepresentation(this->closestObstacleNodes(), resolution);
}

bool Path::is_initialized()
{
	return this->initialized;
}

IndexedNodeArr Path::closestObstacleNodes()
{
	if (this->obstacle_nodes.size() > 0) {
		return this->obstacle_nodes;
	}
	else if (this->nodes.size() > 2) {
		NodeArr waypoints = this->cornerNodes();
		NodeArr obstacles;
		for (unsigned int i = 1; i < waypoints.size()-1; i++) {
			Node prev_waypoint = waypoints.at(i-1);
			Node curr_waypoint = waypoints.at(i);
			Node next_waypoint = waypoints.at(i+1);
			
			int x = curr_waypoint.x;
			int y = curr_waypoint.y;
			NodeArr test_points;
			if (x > 0 && this->map.at(x-1, y) > OCC_THRESHOLD) {
				test_points.push_back(Node(x-1, y));
			}
			if (x < this->map.width-1 && this->map.at(x+1, y) > OCC_THRESHOLD) {
				test_points.push_back(Node(x+1, y));
			}
			if (y > 0 && this->map.at(x, y-1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x, y-1));
			}
			if (y < this->map.height-1 && this->map.at(x, y+1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x, y+1));
			}
			if (x > 0 && y > 0 && this->map.at(x-1, y-1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x-1, y-1));
			}
			if (x < this->map.width-1 && y < this->map.height-1 && this->map.at(x+1, y+1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x+1, y+1));
			}
			if (x > 0 && y < this->map.height-1 && this->map.at(x-1, y+1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x-1, y+1));
			}
			if (x < this->map.width-1 && y > 0 && this->map.at(x+1, y-1) > OCC_THRESHOLD) {
				test_points.push_back(Node(x+1, y-1));
			}
			
			double min_dist = DBL_MAX;
			Node closest_node;
			if (test_points.size() > 0) {
				closest_node = test_points.at(0);
				for (NodeArr::iterator it = test_points.begin(); it < test_points.end(); it++) {
					Node test_point = *it;
					double dist = distance(prev_waypoint, test_point)+distance(curr_waypoint, test_point)+distance(next_waypoint, test_point);
					if (dist < min_dist) {
						min_dist = dist;
						closest_node = test_point;
					}
				}
				IndexedNode t;
				t.node = closest_node;
				
				//std::stringstream sstr;
				//sstr << curr_waypoint << " is " << closest_node;
				//ROS_INFO("Closest node for %s", sstr.str().c_str());
				
				
				
				t.index = i;
				this->obstacle_nodes.push_back(t);
			}
		}
	}
	return this->obstacle_nodes;
}

NodeArr Path::allNodes()
{
	return this->nodes;
}

NodeArr Path::cornerNodes()
{
	if (this->corner_nodes.empty() && this->nodes.size() > 1) {
		//ROS_INFO("%d nodes (without current position %d)", (int)this->nodes.size(), (int)this->nodes.size()-1);
		NodeArr graph = this->removeStraightPathWaypoints(this->nodes);		
		
		//ROS_INFO("After removing straight path nodes %d", (int)graph.size());
		int currentWaypointIdx = graph.size()-1;
		int finalWaypointIdx = 0;
		Node currentWaypoint = graph.at(currentWaypointIdx);
		Node finalWaypoint = graph.at(finalWaypointIdx);
		Node furthestWaypoint = finalWaypoint;
		//ROS_INFO("Current waypoint %d, final %d", currentWaypointIdx, finalWaypointIdx);
		int furthesWaypointIdx = finalWaypointIdx;
		NodeArr newGraph;
	    std::stringstream sstr;
		while (currentWaypoint != furthestWaypoint) {
			sstr.str(std::string());
			sstr << currentWaypoint;
			sstr << "->" << furthestWaypoint;
			bool direct = !(this->isEssentialNodeInBetween(currentWaypointIdx, furthesWaypointIdx, graph) ||
			 this->isObstacleBetweenNodes(currentWaypoint, furthestWaypoint));
			if (!direct) {
			//	ROS_INFO("%s crosses obstacles", sstr.str().c_str());
				if (++furthesWaypointIdx == currentWaypointIdx-1) {
					sstr.str(std::string());
					sstr << currentWaypoint;
				//	ROS_INFO("Saving %s", sstr.str().c_str());
					newGraph.push_back(currentWaypoint);
					currentWaypoint = graph.at(--currentWaypointIdx);
					furthestWaypoint = finalWaypoint;
					furthesWaypointIdx = finalWaypointIdx;
					sstr.str(std::string());
					sstr << currentWaypoint;
				//	ROS_INFO("No more intermediate waypoints. Jumping to %s", sstr.str().c_str());
				}
				else {
					furthestWaypoint = graph.at(furthesWaypointIdx);
				}
			}
			else {
			//	ROS_INFO("%s doesn't cross obstacles", sstr.str().c_str());
				newGraph.push_back(currentWaypoint);
				currentWaypoint = furthestWaypoint;
				currentWaypointIdx = furthesWaypointIdx;
				furthestWaypoint = finalWaypoint;
				furthesWaypointIdx = finalWaypointIdx;
				sstr.str(std::string());
				sstr << currentWaypoint;
			//	ROS_INFO("Jumping to %s", sstr.str().c_str());
			}
		}
		newGraph.push_back(finalWaypoint);
		std::reverse(newGraph.begin(), newGraph.end());
		this->corner_nodes = newGraph;
	}
	return this->corner_nodes;
}

//-------------------- PRIVATE METHODS --------------------------//

bool Path::isEssentialNodeInBetween(int node1_idx, int node2_idx, NodeArr arr)
{
	while (node1_idx > node2_idx) {
		node1_idx--;
		Node node = arr.at(node1_idx);
		if (node.essential) {
			return true;
		}
	}
	return false;
}

bool Path::isObstacleBetweenNodes(Node node1, Node node2)
{
	if (node1 == node2) return false;
	
	//std::stringstream sstr;
	//sstr << node1 << " and " << node2;
	//ROS_INFO("Checking obstacles between %s", sstr.str().c_str());
	
	int denom = node2.x-node1.x;
	if (denom == 0) {
		//Line is vertical
		int begin = std::min(node1.y, node2.y);
		int end = std::max(node1.y, node2.y);
		for (int y = begin+1; y < end; y++) {
			int8_t occupancy = this->map.at(node1.x, y);
			//ROS_INFO("Occupancy of (%d,%d) is %d", node1.x, y, occupancy);
			if (occupancy > OCC_THRESHOLD) {
				return true;
			}
		}
	}
	else {
		Line l = CreateLine(node1.x, node1.y, node2.x, node2.y);
		
		double k = (node2.y-node1.y)/((float)denom);
		double b = node1.y-k*node1.x;
		
		//ROS_INFO("Line K=%.2f B=%.2f", k, b);
		
		int start, finish;
		if (fabs(k) > 1) {
			if (node2.y > node1.y) {
				start = node1.y+1;
				finish = node2.y;
			}
			else {
				start = node2.y+1;
				finish = node1.y;
			}
			for (int i = start; i < finish; i++) {
				int x1 = floor((i-b)/k);
				int x2 = ceil((i-b)/k);
				int8_t occupancy = 0;
				if (this->lineIntersectsNodeAt(l, x1, i)) {
					occupancy = this->map.at(x1, i);
					//ROS_INFO("Occupancy of (%d,%d) is %d", x1, i, occupancy);
				}
				if (occupancy <= OCC_THRESHOLD) {
					if (this->lineIntersectsNodeAt(l, x2, i)) {
						occupancy = this->map.at(x2, i);
						//ROS_INFO("Occupancy of (%d,%d) is %d", x2, i, occupancy);
					}
				}
					
				if (occupancy > OCC_THRESHOLD) {
					return true;
				}
			}
		}
		else {
			if (node2.x > node1.x) {
				start = node1.x+1;
				finish = node2.x;
			}
			else {
				start = node2.x+1;
				finish = node1.x;
			}
			for (int i = start; i < finish; i++) {
				int y1 = floor(k*i+b);
				int y2 = ceil(k*i+b);
				
				int8_t occupancy = 0;
				if (this->lineIntersectsNodeAt(l, i, y1)) {
					occupancy = this->map.at(i, y1);
					//ROS_INFO("Occupancy of (%d,%d) is %d", i, y1, occupancy);
				}
				if (occupancy <= OCC_THRESHOLD) {
					if (this->lineIntersectsNodeAt(l, i, y2)) {
						occupancy = this->map.at(i, y2);
						//ROS_INFO("Occupancy of (%d,%d) is %d", i, y2, occupancy);
					}
				}
				
				if (occupancy > OCC_THRESHOLD) {
					return true;
				}
			}
		}
	}
	return false;	
}

PointArr Path::pointRepresentation(NodeArr graph, float resolution)
{
	PointArr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		Node node = graph.at(i);
		Point point = CreatePoint(node.x*resolution, node.y*resolution);
		points.push_back(point);
	}
	return points;
}

IndexedPointArr Path::pointRepresentation(IndexedNodeArr graph, float resolution)
{
	IndexedPointArr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		IndexedNode node = graph.at(i);
		IndexedPoint point;
		point.point = CreatePoint(node.node.x*resolution, node.node.y*resolution);
		point.index = node.index;
		points.push_back(point);
	}
	return points;
}

bool Path::lineIntersectsNodeAt(Line line, int x, int y)
{
	Point lt = CreatePoint(x-0.5, y-0.5);
	Point rt = CreatePoint(x+0.5, y-0.5);
	Point lb = CreatePoint(x-0.5, y+0.5);
	Point rb = CreatePoint(x+0.5, y+0.5);
	
	Line leftEdge = CreateLine(lt, lb);
	Line rightEdge = CreateLine(rt, rb);
	Line topEdge = CreateLine(lt, rt);
	Line bottomEdge = CreateLine(lb, rb);
	
	return this->linesIntersect(line, leftEdge) ||
		   this->linesIntersect(line, rightEdge) ||
		   this->linesIntersect(line, topEdge) ||
		   this->linesIntersect(line, bottomEdge);
}


bool Path::linesIntersect(Line line1, Line line2)
{
	float q = (line1.p1.y - line2.p1.y) * (line2.p2.x - line2.p1.x) - (line1.p1.x - line2.p1.x) * (line2.p2.y - line2.p1.y);
	float d = (line1.p2.x - line1.p1.x) * (line2.p2.y - line2.p1.y) - (line1.p2.y - line1.p1.y) * (line2.p2.x - line2.p1.x);

	if (d == 0)
	{
		return false;
	}

	float r = q / d;

	q = (line1.p1.y - line2.p1.y) * (line1.p2.x - line1.p1.x) - (line1.p1.x - line2.p1.x) * (line1.p2.y - line1.p1.y);
	float s = q / d;

	if (r < 0 || r > 1 || s < 0 || s > 1)
	{
		return false;
	}

	return true;
}

bool Path::in_set(NodeList set, Node node) {
	NodeList::iterator it = find(set.begin(), set.end(), node);
	return it != set.end();
}
	
double Path::distance(Node node1, Node node2) {
    if (node1.x >= 0 && node1.x < this->map.width && 
       node2.x >= 0 && node2.x < this->map.width && 
       node1.y >=0 && node1.y < this->map.height && 
       node2.y >= 0 && node2.y < this->map.height) {
		int dx = node1.x-node2.x;
		int dy = node1.y-node2.y;
		return sqrt(pow(dx, 2) + pow(dy, 2));
    }
    return this->map.width*this->map.height;
}

NodeArr Path::removeStraightPathWaypoints(NodeArr originalGraph)
{
	NodeArr newGraph;
	NodeArr::iterator it = originalGraph.begin();
	if (originalGraph.size() > 0) {
		newGraph.push_back(*it);
	}
	for (unsigned int i = 1; i < originalGraph.size()-1; i++) {
		Node node = originalGraph.at(i);
		bool needsToBeAdded = node.essential;
		if (!needsToBeAdded) {
			Node previousNode = originalGraph.at(i-1);
			Node nextNode = originalGraph.at(i+1);
			//ROS_INFO("Comparing (%d-%d) vs %d, (%d-%d) vs %d", nextNode.x, previousNode.x, node.x, nextNode.y, previousNode.y, node.y);
			needsToBeAdded = !((nextNode.x+previousNode.x)/2.0 == node.x && (nextNode.y+previousNode.y)/2.0 == node.y);
		}
		if (needsToBeAdded) {
			newGraph.push_back(node);
		}
		else {
			std::stringstream sstr;
			sstr << node;
		}
	}
	if (originalGraph.size() > 1) {
		it = originalGraph.end()-1;
		newGraph.push_back(*it);
	}
	return newGraph;
}


NodeArr Path::reconstruct_path(NodeList came_from, Node current)
{	
	NodeArr path;
	Node parent = current.parent(came_from);
	//std::stringstream sstr;
	//sstr << parent << " of " << current;
	//ROS_INFO("Parent %s", sstr.str().c_str());
	if (parent != current) {
		path = this->reconstruct_path(came_from, parent);
		path.insert(path.begin(), current);
	}
	else {
		path.push_back(current);
	}
	return path;
}

//-------------------- OTHER METHODS --------------------------//

void lunabotics::PrintNodes(NodeArr arr, std::string label)
{
	ROS_INFO("==============%s===============", label.c_str());
    std::stringstream sstr;
    for (NodeArr::iterator it = arr.begin(); it < arr.end(); it++) {
		sstr << (*it) << std::endl;
	}
	ROS_INFO("%s", sstr.str().c_str());
	ROS_INFO("======================================");
}
