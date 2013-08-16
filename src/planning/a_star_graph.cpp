/* Check http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode for details */
#include "ros/ros.h"
#include "a_star_graph.h"
#include <math.h>
#include <list>
#include <sstream>
#include <algorithm>
#include "float.h"
#include "../geometry/basic.h"
using namespace lunabotics;

//---------------------------- CONSTRUCTOR / DESCTRUCTOR ------------------------//
Path::Path(): map(), initialized(false), nodes(), points()
{
}

Path::Path(MapData map, Point start): 
map(map), 
initialized(true), 
use_cspace(false), 
nodes(), 
points(), 
robot_dimensions(),
initialOrientation(0)
{
	//ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start.x, start.y, goal.x, goal.y);	
	if (map.at(start) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); this->initialized = false; }
    
    this->nodes.push_back(Node(start));
}

Path::Path(MapData map, Point start, Rect robotDimensions): 
map(map), 
initialized(true), 
use_cspace(true), 
nodes(),
points(), 
robot_dimensions(robotDimensions),
initialOrientation(0)
{
	//ROS_INFO("Looking for a path (%d,%d)->(%d,%d)", start.x, start.y, goal.x, goal.y);	
	if (map.at(start) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); this->initialized = false; }
    
    this->nodes.push_back(Node(start));
}

Path::Path(MapData map, Point start, Rect robotDimensions, bool useCSpace, double initialOrientation):
map(map),
initialized(true), 
use_cspace(useCSpace),
nodes(), 
points(), 
robot_dimensions(robotDimensions),
initialOrientation(initialOrientation)
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
			
			//ROS_INFO("Picking (%d,%d). G = %f", current.x, current.y, current.getG());
			
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
					
	        
	        NodeList neighbours =// this->use_cspace ? current.neighbours(this->map, this->robot_dimensions) : 
	        current.neighbours(this->map);
	        
	        for (NodeList::iterator it = neighbours.begin(); it != neighbours.end(); it++) {
				Node neighbour = *it;
				
				if (this->in_set(closed_set, neighbour)) {
					continue;
				}
				
				double obstacle_dist;
				int safe_radius = ceil(fabs(this->robot_dimensions.left_front.x)/this->map.resolution);
				double repulsion_penalty = 0;
				if (this->obstacles_in_circle(neighbour, safe_radius, obstacle_dist)) {
					repulsion_penalty = (safe_radius*5)/obstacle_dist;
					//ROS_WARN("Obstacle at distance %f. Penalty %f (Safe radius is %d cells)", obstacle_dist, repulsion_penalty, safe_radius);
				}
				
				double tentative_G = current.getG() + this->distance(current, neighbour) + repulsion_penalty;
				
				//ROS_INFO("Dist from beginning %f, curr G %f, tentative G %f <> %f", this->distance(current, neighbour), current.getG(), tentative_G, neighbour.getG());
				
				//Node original;
				//if (this->in_set(closed_set, neighbour, original)) {
					//if (original.getG() <= tentative_G) {
						//continue;
					//}
				//}
				
				if (!this->in_set(open_set, neighbour) || tentative_G <= neighbour.getG()) {
					if (!this->in_set(came_from, current)) {
						came_from.push_back(current);
					}
					
					neighbour.setParent(current);
					neighbour.setG(tentative_G);
					neighbour.setH(this->distance(neighbour, goal_node));
					
			        if (came_from.size() >= 3) {
						Node secondPrevious = current.parent(came_from);
						if (secondPrevious != current) {
							bool direct = !current.essential && !this->isObstacleBetweenNodes(secondPrevious, neighbour);
									
							if (this->use_cspace) { 
							  //Check if this transition will be possible at required orientation (or if orientation is not changed)
							  double orient1 = atan2((neighbour.y-current.y), (neighbour.x-current.x));
							  double orient2 = atan2((current.y-secondPrevious.y), (current.x-secondPrevious.x));
							  
							  if (came_from.size() == 1) {
								  //If just starting and rotating in place. Check that we don't bump into smth while rotating
								  //Checking the orientation half-way
								  double orient3 = normalizedAngle((orient1+this->initialOrientation)/2);
								  direct = direct && (fabs(orient3-orient1) < 0.0001 || robotFitsAtNode(current.x, current.y, orient3, this->robot_dimensions, this->map));
							  }
							  
							  direct = direct && (secondPrevious.isPossible(neighbour, this->robot_dimensions, this->map) || fabs(orient2-orient1) < 0.0001);
							}
							
							if (direct) {
								std::stringstream sstr;
								sstr << secondPrevious << " and " << neighbour << ". Removing " << current;
								//ROS_INFO("Direct connection between %s", sstr.str().c_str());
								neighbour.setParent(secondPrevious);
							}
						}
					}
					
					//Need updated node here
					if (this->in_set(open_set, neighbour)) {
						open_set.remove(neighbour);
					}
					open_set.push_back(neighbour);
					
					
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

PointArr Path::allPoints()
{	
	if (this->points.size() < 1) {
		this->points = this->pointRepresentation(this->allNodes());
	}
	return this->points;
}

IndexedPointArr Path::closestObstaclePoints()
{
	if (this->obstacle_points.size() > 0) {
		return this->obstacle_points;
	}
	else if (this->allNodes().size() > 2) {
		PointArr obstacles;
		for (unsigned int i = 1; i < this->allNodes().size()-1; i++) {
			Node prev_node = this->allNodes().at(i-1);
			Node curr_node = this->allNodes().at(i);
			Node next_node = this->allNodes().at(i+1);
						
			Point obstacle;
			if (this->obstacle_in_triangle(prev_node, curr_node, next_node, obstacle)) {
				IndexedPoint t;
				t.point = obstacle*this->map.resolution;
				t.index = i;
				
				std::stringstream sstr;
				sstr << curr_node;
				ROS_INFO("Closest point for %s is (%.2f,%.2f) (Index %d)", sstr.str().c_str(), obstacle.x, obstacle.y, i);
				
				this->obstacle_points.push_back(t);
			}
		}
	}
	return this->obstacle_points;
}

bool Path::is_initialized()
{
	return this->initialized;
}

NodeArr Path::allNodes()
{
	return this->nodes;
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

PointArr Path::pointRepresentation(NodeArr graph)
{
	PointArr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		Node node = graph.at(i);
		Point point = Point_from_Node(node)*this->map.resolution;
		points.push_back(point);
	}
	return points;
}

IndexedPointArr Path::pointRepresentation(IndexedNodeArr graph)
{
	IndexedPointArr points;
	for (unsigned int i = 0; i < graph.size(); i++) {
		IndexedNode node = graph.at(i);
		IndexedPoint point;
		point.point = CreatePoint(node.node.x*this->map.resolution, node.node.y*this->map.resolution);
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
	
	
	/*
	Point lt = CreatePoint(x-0.5, y+0.5);
	Point rt = CreatePoint(x+0.5, y+0.5);
	Point lb = CreatePoint(x-0.5, y-0.5);
	Point rb = CreatePoint(x+0.5, y-0.5);
	
	Rect s = CreateRect(lt, rt, lb, rb);
	return line_crosses_square(line, s);*/
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

bool Path::in_set(NodeList set, Node node, Node &original) {
	NodeList::iterator it = find(set.begin(), set.end(), node);
	bool result = it != set.end();
	if (result) {
		original = *it;
	}
	return result;
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

bool Path::obstacles_in_circle(Node center, int radius, double &closestDistance)
{
	bool result = false;
	closestDistance = -1;
	for (int i = std::max(0, center.y-radius); i < center.y+radius && i < this->map.height; i++) {
		for (int j = std::max(0, center.x-radius); j < center.x+radius && j < this->map.width; j++) {
			if (in_circle(CreatePoint(j, i), Point_from_Node(center), radius) && this->map.at(j, i) > OCC_THRESHOLD) {
				Node n = Node(j, i);
				double dist = this->distance(n, center);
				if (closestDistance == -1 || dist < closestDistance) {
					closestDistance = dist;
					result = true;
				}				
			}
		}
	}
	return result;
}

//P2 being a corner node
bool Path::obstacle_in_triangle(Node n1, Node n2, Node n3, Point &obstacle)
{
	bool result = false;
	double closestDistance = -1;
	double closestDistToEdge = -1;
	int min_x = std::min(std::min(n1.x, n2.x), n3.x);
	int min_y = std::min(std::min(n1.y, n2.y), n3.y);
	int max_x = std::max(std::max(n1.x, n2.x), n3.x);
	int max_y = std::max(std::max(n1.y, n2.y), n3.y);
	Triangle t = CreateTriangle(Point_from_Node(n1), Point_from_Node(n2), Point_from_Node(n3));
	
	Point p1 = CreatePoint(0.5, 0.5);
	Point p2 = CreatePoint(0.5, -0.5);
		
	for (int i = min_y; i <= max_y; i++) {
		for (int j = min_x; j <= max_x; j++) {
			Point test_point = CreatePoint(j, i);
			if (this->map.at(j, i) > OCC_THRESHOLD && in_triangle(t, test_point)) {
				//ROS_WARN("Node %d,%d is in bezier triangle", j, i);
				
				//Since obstacle is the center of occupied cell, we want p to be at its edge
				Point corners[] = {test_point+p1,test_point-p1,test_point+p2,test_point-p2};
				Point closestPoint;
				for (unsigned int ci = 0; ci < 4; ci++) {
					//double dist_to_path_1 = heightOfTriangle(CreateTriangle(t.p1, corners[ci], t.p2), false);
					//double dist_to_path_2 = heightOfTriangle(CreateTriangle(t.p2, corners[ci], t.p3), false);
					double dist = //dist_to_path_1+dist_to_path_2+distance(corners[ci], t.p2);
					
					distance(corners[ci], t.p2);
					if (closestDistance == -1 || (closestDistance-dist) > 0.0001) {
						closestDistance = dist;
						closestDistToEdge = std::min(heightOfTriangle(CreateTriangle(t.p1, corners[ci], t.p2), false), heightOfTriangle(CreateTriangle(t.p2, corners[ci], t.p3), false));
						obstacle = corners[ci];
						result = true;
					}
					else if ((closestDistance-dist) <= 0.0001) {
						//Should be closer to the longer edge
						double distToEdge = std::min(heightOfTriangle(CreateTriangle(t.p1, corners[ci], t.p2), false), heightOfTriangle(CreateTriangle(t.p2, corners[ci], t.p3), false));
							
						//ROS_WARN("Same distance obstacle");
						if (distToEdge < closestDistToEdge) {
							//ROS_WARN("%.1f,%.1f is closer to the longer edge than %.1f,%.1f (%.3f vs %.3f)", corners[ci].x, corners[ci].y, obstacle.x, obstacle.y, distToEdge, closestDistToEdge);
							closestDistToEdge = distToEdge;
							obstacle = corners[ci];
							result = true;
						}	
						//else {
							//ROS_WARN("%.1f,%.1f is NOT closer to the longer edge than %.1f,%.1f,  (%.3f vs %.3f)", corners[ci].x, corners[ci].y, obstacle.x, obstacle.y, distToEdge, closestDistToEdge);
						//}
					}
				}
				//ROS_INFO("Closest dist is %.2f from %.1f,%.1f", closestDistance, obstacle.x, obstacle.y);
			}
		}
	}
	return result;
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
