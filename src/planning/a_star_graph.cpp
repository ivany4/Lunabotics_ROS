/* Check http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode for details */
#include "ros/ros.h"
#include "a_star_graph.h"
#include <math.h>
#include <list>
#include <sstream>
#include <algorithm>
#include "float.h"
using namespace lunabotics;
using namespace lunabotics::planning;

//---------------------------- CONSTRUCTOR / DESCTRUCTOR ------------------------//
path::path(): width(0), height(0), initialized(false), map(), nodes(), corner_nodes()
{
}

path::path(map_grid map, int width, int height, int start_x, int start_y, int goal_x, int goal_y): width(width), height(height), initialized(true), map(map), nodes(), corner_nodes()
{
	//ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start_x, start_y, goal_x, goal_y);	
	node_arr graph;
	
    if (map.at(goal_x+goal_y*width) > OCC_THRESHOLD) { ROS_ERROR("Goal cell is occupied"); this->initialized = false; }
    if (map.at(start_x+start_y*width) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); this->initialized = false; }
    
    if (this->initialized) {
		node_list open_set;
		node_list closed_set;
		node_list came_from;
		
	    this->width = width;
	    this->height = height;
	    
	    node goal_node = node(goal_x, goal_y);  
		
	      
	    node start_node = node(start_x, start_y);
	    start_node.H = this->distance(start_node, goal_node);	//Distance
	    start_node.G = 0;										//Cost
	    start_node.F = start_node.H+start_node.G;				//Admissible heuristics
	    start_node.parent_x = start_x;
	    start_node.parent_y = start_y;
	    open_set.push_back(start_node);
	    
	    std::stringstream sstr;
	    
	    while (!open_set.empty()) {
	        open_set.sort();
			node current = open_set.front();
			sstr.str(std::string());
			sstr << current;
			//ROS_INFO("Node with smallest F=%.3f is %s", current.F, sstr.str().c_str());
	
	        if (current == goal_node) {
				//ROS_INFO("Found goal %s", sstr.str().c_str());
				graph = this->reconstruct_path(came_from, current);
				reverse(graph.begin(), graph.end());
				break;
			}
			
	        open_set.remove(current);
	        closed_set.push_back(current);
	        
	        node_list neighbours = current.neighbours(this->width, this->height, map);
	        
	        for (node_list::iterator it = neighbours.begin(); it != neighbours.end(); it++) {
				node neighbour = *it;
				if (this->in_set(closed_set, neighbour)) {
					continue;
				}
				
				double tentative_G = current.G + this->distance(current, neighbour);
				
				if (!this->in_set(open_set, neighbour) || tentative_G <= neighbour.G) {
					came_from.push_back(current);
					neighbour.parent_x = current.x;
					neighbour.parent_y = current.y;
					neighbour.G = tentative_G;
					neighbour.H = this->distance(neighbour, goal_node);
					neighbour.F = neighbour.G + neighbour.H;
					if (!this->in_set(open_set, neighbour)) {
						open_set.push_back(neighbour);
					}
				}
			}
	    }
	
		this->nodes = graph;
	}
}

//-------------------- GETTER / SETTER --------------------------//

point_arr path::cornerPoints(float resolution)
{
	return this->pointRepresentation(this->cornerNodes(), resolution);
}

point_arr path::allPoints(float resolution)
{
	return this->pointRepresentation(this->allNodes(), resolution);
}

point_indexed_arr path::closestObstaclePoints(float resolution)
{
	return this->pointRepresentation(this->closestObstacleNodes(), resolution);
}

bool path::is_initialized()
{
	return this->initialized;
}

node_indexed_arr path::closestObstacleNodes()
{
	if (this->obstacle_nodes.size() > 0) {
		return this->obstacle_nodes;
	}
	else if (this->nodes.size() > 2) {
		node_arr waypoints = this->cornerNodes();
		node_arr obstacles;
		for (unsigned int i = 1; i < waypoints.size()-1; i++) {
			node prev_waypoint = waypoints.at(i-1);
			node curr_waypoint = waypoints.at(i);
			node next_waypoint = waypoints.at(i+1);
			
			int x = curr_waypoint.x;
			int y = curr_waypoint.y;
			node_arr test_points;
			if (x > 0 && this->mapAt(x-1, y) > OCC_THRESHOLD) {
				test_points.push_back(node(x-1, y));
			}
			if (x < this->width-1 && this->mapAt(x+1, y) > OCC_THRESHOLD) {
				test_points.push_back(node(x+1, y));
			}
			if (y > 0 && this->mapAt(x, y-1) > OCC_THRESHOLD) {
				test_points.push_back(node(x, y-1));
			}
			if (y < this->height-1 && this->mapAt(x, y+1) > OCC_THRESHOLD) {
				test_points.push_back(node(x, y+1));
			}
			if (x > 0 && y > 0 && this->mapAt(x-1, y-1) > OCC_THRESHOLD) {
				test_points.push_back(node(x-1, y-1));
			}
			if (x < this->width-1 && y < this->height-1 && this->mapAt(x+1, y+1) > OCC_THRESHOLD) {
				test_points.push_back(node(x+1, y+1));
			}
			if (x > 0 && y < this->height-1 && this->mapAt(x-1, y+1) > OCC_THRESHOLD) {
				test_points.push_back(node(x-1, y+1));
			}
			if (x < this->width-1 && y > 0 && this->mapAt(x+1, y-1) > OCC_THRESHOLD) {
				test_points.push_back(node(x+1, y-1));
			}
			
			double min_dist = DBL_MAX;
			node closest_node;
			if (test_points.size() > 0) {
				closest_node = test_points.at(0);
				for (node_arr::iterator it = test_points.begin(); it < test_points.end(); it++) {
					node test_point = *it;
					double dist = distance(prev_waypoint, test_point)+distance(curr_waypoint, test_point)+distance(next_waypoint, test_point);
					if (dist < min_dist) {
						min_dist = dist;
						closest_node = test_point;
					}
				}
				node_indexed t;
				t.node = closest_node;
				t.index = i;
				this->obstacle_nodes.push_back(t);
			}
		}
	}
	return this->obstacle_nodes;
}

node_arr path::allNodes()
{
	return this->nodes;
}

node_arr path::cornerNodes()
{
	if (this->corner_nodes.size() > 0) {
		return this->corner_nodes;
	}
	else if (this->nodes.size() > 1) {
		//ROS_INFO("%d nodes (without current position %d)", (int)this->nodes.size(), (int)this->nodes.size()-1);
		node_arr graph = this->removeStraightPathWaypoints(this->nodes);		
		//ROS_INFO("After removing straight path nodes %d", (int)graph.size());
		int currentWaypointIdx = graph.size()-1;
		int finalWaypointIdx = 0;
		node currentWaypoint = graph.at(currentWaypointIdx);
		node finalWaypoint = graph.at(finalWaypointIdx);
		node furthestWaypoint = finalWaypoint;
		//ROS_INFO("Current waypoint %d, final %d", currentWaypointIdx, finalWaypointIdx);
		int furthesWaypointIdx = finalWaypointIdx;
		node_arr newGraph;
	    std::stringstream sstr;
		while (currentWaypoint != furthestWaypoint) {
			sstr.str(std::string());
			sstr << currentWaypoint;
			sstr << "->" << furthestWaypoint;
			bool crossesObstacles = this->isObstacleBetweenNodes(currentWaypoint, furthestWaypoint);
			if (crossesObstacles) {
				//ROS_INFO("%s crosses obstacles", sstr.str().c_str());
				if (++furthesWaypointIdx == currentWaypointIdx-1) {
					sstr.str(std::string());
					sstr << currentWaypoint;
					ROS_INFO("Saving %s", sstr.str().c_str());
					newGraph.push_back(currentWaypoint);
					currentWaypoint = graph.at(--currentWaypointIdx);
					furthestWaypoint = finalWaypoint;
					furthesWaypointIdx = finalWaypointIdx;
					sstr.str(std::string());
					sstr << currentWaypoint;
					//ROS_INFO("No more intermediate waypoints. Jumping to %s", sstr.str().c_str());
				}
				else {
					furthestWaypoint = graph.at(furthesWaypointIdx);
				}
			}
			else {
				//ROS_INFO("%s doesn't cross obstacles", sstr.str().c_str());
				newGraph.push_back(currentWaypoint);
				currentWaypoint = furthestWaypoint;
				currentWaypointIdx = furthesWaypointIdx;
				furthestWaypoint = finalWaypoint;
				furthesWaypointIdx = finalWaypointIdx;
				sstr.str(std::string());
				sstr << currentWaypoint;
				//ROS_INFO("Jumping to %s", sstr.str().c_str());
			}
		}
		newGraph.push_back(finalWaypoint);
		std::reverse(newGraph.begin(), newGraph.end());
		this->corner_nodes = newGraph;
		return newGraph;
	}
	return this->corner_nodes;
}

//-------------------- PRIVATE METHODS --------------------------//

bool path::isObstacleBetweenNodes(node node1, node node2)
{
	if (node1 == node2) return false;
	
	std::stringstream sstr;
	sstr << node1 << " and " << node2;
	ROS_INFO("Checking obstacles between %s", sstr.str().c_str());
	
	int denom = node2.x-node1.x;
	if (denom == 0) {
		//Line is vertical
		int begin = std::min(node1.y, node2.y);
		int end = std::max(node1.y, node2.y);
		for (int y = begin+1; y < end; y++) {
			int8_t occupancy = this->mapAt(node1.x, y);
			ROS_INFO("Occupancy of (%d,%d) is %d", node1.x, y, occupancy);
			if (occupancy > OCC_THRESHOLD) {
				return true;
			}
		}
	}
	else {
		double k = (node2.y-node1.y)/((float)denom);
		double b = node1.y-k*node1.x;
		
		ROS_INFO("Line K=%.2f B=%.2f", k, b);
		
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
				int x = round((i-b)/k);
				int8_t occupancy = this->mapAt(x, i);
				ROS_INFO("Occupancy of (%d,%d) is %d", x, i, occupancy);
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
				int y = round(k*i+b);
				int8_t occupancy = this->mapAt(i, y);
				ROS_INFO("Occupancy of (%d,%d) is %d", i, y, occupancy);
				if (occupancy > OCC_THRESHOLD) {
					return true;
				}
			}
		}
	}
	return false;	
}

point_arr path::pointRepresentation(node_arr graph, float resolution)
{
	point_arr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		node node = graph.at(i);
		point_t point;
		point.x = node.x*resolution;
		point.y = node.y*resolution;
		points.push_back(point);
	}
	return points;
}

point_indexed_arr path::pointRepresentation(node_indexed_arr graph, float resolution)
{
	point_indexed_arr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		node_indexed node = graph.at(i);
		point_indexed point;
		point.point.x = node.node.x*resolution;
		point.point.y = node.node.y*resolution;
		point.index = node.index;
		points.push_back(point);
	}
	return points;
}


bool path::in_set(node_list set, node node) {
	node_list::iterator it = find(set.begin(), set.end(), node);
	return it != set.end();
}
	
double path::distance(node node1, node node2) {
    if (node1.x >= 0 && node1.x < this->width && 
       node2.x >= 0 && node2.x < this->width && 
       node1.y >=0 && node1.y < this->height && 
       node2.y >= 0 && node2.y < this->height) {
		int dx = node1.x-node2.x;
		int dy = node1.y-node2.y;
		return sqrt(pow(dx, 2) + pow(dy, 2));
    }
    return this->width*this->height;
}

node_arr path::removeStraightPathWaypoints(node_arr originalGraph)
{
	node_arr newGraph;
	node_arr::iterator it = originalGraph.begin();
	if (originalGraph.size() > 0) {
		newGraph.push_back(*it);
	}
	for (unsigned int i = 1; i < originalGraph.size()-1; i++) {
		node previousNode = originalGraph.at(i-1);
		node nextNode = originalGraph.at(i+1);
		node node = originalGraph.at(i);
		//ROS_INFO("Comparing (%d-%d) vs %d, (%d-%d) vs %d", nextNode.x, previousNode.x, node.x, nextNode.y, previousNode.y, node.y);
		if (!((nextNode.x+previousNode.x)/2.0 == node.x && (nextNode.y+previousNode.y)/2.0 == node.y)) {
			newGraph.push_back(node);
		}
	}
	if (originalGraph.size() > 1) {
		it = originalGraph.end()-1;
		newGraph.push_back(*it);
	}
	return newGraph;
}


node_arr path::reconstruct_path(node_list came_from, node current)
{	
	node_arr path;
	node parent = current.parent(came_from);
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

int8_t path::mapAt(int x, int y)
{
	unsigned int idx = width*y+x;
	if (idx >= this->map.size()) {
		ROS_ERROR("Trying to get cell beyond the map boundaries");
		return 0;
	}
	return this->map.at(width*y+x);
}
