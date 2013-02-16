/* Check http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode for details */
#include "ros/ros.h"
#include "a_star_graph.h"
#include <math.h>
#include <list>
#include <sstream>
using namespace std;

vector<a_star_node> a_star_graph::find_path(vector<int8_t> map, int width, int height, int start_x, int start_y, int goal_x, int goal_y)
{
	ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start_x, start_y, goal_x, goal_y);	
	vector<a_star_node> graph;
    if (map.at(goal_x+goal_y*width) > OCC_THRESHOLD) { ROS_ERROR("Goal cell is occupied"); return graph; }
    if (map.at(start_x+start_y*width) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); return graph; }
    
    
	list<a_star_node> open_set;
	list<a_star_node> closed_set;
	list<a_star_node> came_from;
	
    this->width = width;
    this->height = height;
    
    a_star_node goal_node = a_star_node(goal_x, goal_y);  
	
      
    a_star_node start_node = a_star_node(start_x, start_y);
    start_node.H = this->distance(start_node, goal_node);	//Distance
    start_node.G = 0;										//Cost
    start_node.F = start_node.H+start_node.G;				//Admissible heuristics
    start_node.parent_x = start_x;
    start_node.parent_y = start_y;
    open_set.push_back(start_node);
    
    stringstream sstr;
    
    while (!open_set.empty()) {
        open_set.sort();
		a_star_node current = open_set.front();
		sstr.str(string());
		sstr << current;
		ROS_INFO("Node with smallest F=%.3f is %s", current.F, sstr.str().c_str());

        if (current == goal_node) {
			ROS_INFO("Found goal %s", sstr.str().c_str());
			graph = this->reconstruct_path(came_from, current);
			reverse(graph.begin(), graph.end());
			break;
		}
		
        open_set.remove(current);
        closed_set.push_back(current);
        
        list<a_star_node> neighbours = current.neighbours(this->width, this->height, map);
        
        for (list<a_star_node>::iterator it = neighbours.begin(); it != neighbours.end(); it++) {
			a_star_node neighbour = *it;
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

    return graph;
}

vector<a_star_node> a_star_graph::reconstruct_path(list<a_star_node> came_from, a_star_node current)
{	
	vector<a_star_node> path;
	a_star_node parent = current.parent(came_from);
	if (parent != current) {
		path = this->reconstruct_path(came_from, parent);
		path.insert(path.begin(), current);
	}
	else {
		path.push_back(current);
	}
	return path;
}
	
bool a_star_graph::in_set(list<a_star_node> set, a_star_node node) {
	list<a_star_node>::iterator it = find(set.begin(), set.end(), node);
	return it != set.end();
}
	
double a_star_graph::distance(a_star_node node1, a_star_node node2) {
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

vector<a_star_node> a_star_graph::removeIntermediateWaypoints(std::vector<a_star_node> originalGraph)
{
	vector<a_star_node> newGraph;
	vector<a_star_node>::iterator it = originalGraph.begin();
	if (originalGraph.size() > 0) {
		newGraph.push_back(*it);
	}
	for (unsigned int i = 1; i < originalGraph.size()-1; i++) {
		a_star_node previousNode = originalGraph.at(i-1);
		a_star_node nextNode = originalGraph.at(i+1);
		a_star_node node = originalGraph.at(i);
		ROS_INFO("Comparing (%d-%d) vs %d, (%d-%d) vs %d", nextNode.x, previousNode.x, node.x, nextNode.y, previousNode.y, node.y);
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

vector<a_star_node> a_star_graph::removeIntermediateWaypoints(vector<a_star_node> originalGraph, vector<int8_t> map, int width)
{
	if (originalGraph.size() > 1) {
		ROS_INFO("%d nodes (without current position %d)", originalGraph.size(), originalGraph.size()-1);
		vector<a_star_node> graph = removeIntermediateWaypoints(originalGraph);
		ROS_INFO("After removing straight path nodes %d", graph.size());
		int currentWaypointIdx = 0;
		int finalWaypointIdx = graph.size()-1;
		a_star_node currentWaypoint = graph.at(currentWaypointIdx);
		a_star_node finalWaypoint = graph.at(finalWaypointIdx);
		a_star_node furthestWaypoint = finalWaypoint;
		ROS_INFO("Current waypoint %d, final %d", currentWaypointIdx, finalWaypointIdx);
		int furthesWaypointIdx = finalWaypointIdx;
		vector<a_star_node> newGraph;
	    stringstream sstr;
		while (currentWaypoint != furthestWaypoint) {
			sstr.str(string());
			sstr << currentWaypoint;
			sstr << "->" << furthestWaypoint;
			bool crossesObstacles = this->isObstacleBetweenNodes(currentWaypoint, furthestWaypoint, map, width);
			if (crossesObstacles) {
				ROS_INFO("%s crosses obstacles", sstr.str().c_str());
				if (--furthesWaypointIdx == currentWaypointIdx+1) {
					sstr.str(string());
					sstr << currentWaypoint;
					ROS_INFO("Saving %s", sstr.str().c_str());
					newGraph.push_back(currentWaypoint);
					currentWaypoint = graph.at(++currentWaypointIdx);
					furthestWaypoint = finalWaypoint;
					furthesWaypointIdx = finalWaypointIdx;
					sstr.str(string());
					sstr << currentWaypoint;
					ROS_INFO("No more intermediate waypoints. Jumping to %s", sstr.str().c_str());
				}
				else {
					furthestWaypoint = graph.at(furthesWaypointIdx);
				}
			}
			else {
				ROS_INFO("%s doesn't cross obstacles", sstr.str().c_str());
				newGraph.push_back(currentWaypoint);
				currentWaypoint = furthestWaypoint;
				currentWaypointIdx = furthesWaypointIdx;
				furthestWaypoint = finalWaypoint;
				furthesWaypointIdx = finalWaypointIdx;
				sstr.str(string());
				sstr << currentWaypoint;
				ROS_INFO("Jumping to %s", sstr.str().c_str());
			}
		}
		newGraph.push_back(finalWaypoint);
		return newGraph;
	}
	return originalGraph;
}

a_star_graph::a_star_graph(): width(0), height(0)
{
}

bool a_star_graph::isObstacleBetweenNodes(a_star_node node1, a_star_node node2, vector<int8_t> map, int width)
{
	if (node1 == node2) return false;
	
	stringstream sstr;
	sstr << node1 << " and " << node2;
	ROS_INFO("Checking obstacles between %s", sstr.str().c_str());
	
	int denom = node2.x-node1.x;
	double k = denom != 0 ? (node2.y-node1.y)/denom : 0;
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
			int8_t occupancy = map.at(width*i+x);
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
			int y = k*i+b;
			int8_t occupancy = map.at(width*y+i);
			ROS_INFO("Occupancy of (%d,%d) is %d", i, y, occupancy);
			if (occupancy > OCC_THRESHOLD) {
				return true;
			}
		}
	}
	return false;	
}
