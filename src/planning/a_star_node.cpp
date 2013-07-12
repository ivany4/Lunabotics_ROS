#include "ros/ros.h"
#include "a_star_node.h"
#include "../geometry/basic.h"
#include <iostream>
#include <list>
#include <vector>
using namespace lunabotics;

#define USE_8_DIRECTIONS	1

Node::Node():F(0), G(0), H(0), has_F(false), x(0), y(0), parent_x(0), parent_y(0), essential(false)
{
}

Node::Node(int nx, int ny):F(0), G(0), H(0), has_F(false), x(nx), y(ny), parent_x(0), parent_y(0), essential(false)
{
}

Node::Node(Point p):F(0), G(0), H(0), has_F(false), x(p.x), y(p.y), parent_x(0), parent_y(0), essential(false)
{
}

Node::Node(const Node &copyin):F(copyin.F), G(copyin.G), H(copyin.H), has_F(copyin.has_F),
 x(copyin.x), y(copyin.y), parent_x(copyin.parent_x), parent_y(copyin.parent_y), essential(copyin.essential)
{
}

namespace lunabotics {
std::ostream &operator<<(std::ostream &output, const Node &aaa)
{
   output << "(" << aaa.x << "," << aaa.y << ")";// << " ess " << aaa.essential << ")";
   return output;
}
}

Node& Node::operator=(const Node &rhs)
{
   this->x = rhs.x;
   this->y = rhs.y;
   this->F = rhs.F;
   this->G = rhs.G;
   this->H = rhs.H;
   this->has_F = rhs.has_F;
   this->parent_x = rhs.parent_x;
   this->parent_y = rhs.parent_y;
   this->essential = rhs.essential;
   return *this;
}

int Node::operator==(const Node &rhs) const
{
   if( this->x != rhs.x) return 0;
   if( this->y != rhs.y) return 0;

   return 1;
}

int Node::operator!=(const Node &rhs) const
{
   if( this->x != rhs.x) return 1;
   if( this->y != rhs.y) return 1;

   return 0;
}

NodeList Node::neighbours(MapData map)
{
	NodeList result;
	if (this->x > 0) {
		int new_x = this->x-1;
		if (this->isPossible(new_x, this->y, map)) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y > 0) {
		int new_y = this->y-1;
		if (this->isPossible(this->x, new_y, map)) {
			result.push_back(Node(this->x, new_y));
		}
	}
	if (this->x < map.width-1) {
		int new_x = this->x+1;
		if (this->isPossible(new_x, this->y, map)) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y < map.height-1) {
		int new_y = this->y+1;
		if (this->isPossible(this->x, new_y, map)) {
			result.push_back(Node(this->x, new_y));
		}
	}
#ifdef USE_8_DIRECTIONS
		
	if (this->x > 0 && this->y > 0) {
		int new_x = this->x-1;
		int new_y = this->y-1;
		if (this->isPossible(new_x, new_y, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < map.width-1 && this->y < map.height-1) {
		int new_x = this->x+1;
		int new_y = this->y+1;
		if (this->isPossible(new_x, new_y, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x > 0 && this->y < map.height-1) {
		int new_x = this->x-1;
		int new_y = this->y+1;
		if (this->isPossible(new_x, new_y, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < map.width-1 && this->y > 0) {
		int new_x = this->x+1;
		int new_y = this->y-1;
		if (this->isPossible(new_x, new_y, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
		
#endif


	return result;
}

NodeList Node::neighbours(MapData map, Rect robotDimensions)
{
	NodeList result;
	if (this->x > 0) {
		int new_x = this->x-1;
		if (this->isPossible(new_x, this->y, robotDimensions, map)) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y > 0) {
		int new_y = this->y-1;
		if (this->isPossible(this->x, new_y, robotDimensions, map)) {
			result.push_back(Node(this->x, new_y));
		}
	}
	if (this->x < map.width-1) {
		int new_x = this->x+1;
		if (this->isPossible(new_x, this->y, robotDimensions, map)) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y < map.height-1) {
		int new_y = this->y+1;
		if (this->isPossible(this->x, new_y, robotDimensions, map)) {
			result.push_back(Node(this->x, new_y));
		}
	}
#ifdef USE_8_DIRECTIONS
		
	if (this->x > 0 && this->y > 0) {
		int new_x = this->x-1;
		int new_y = this->y-1;
		if (this->isPossible(new_x, new_y, robotDimensions, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < map.width-1 && this->y < map.height-1) {
		int new_x = this->x+1;
		int new_y = this->y+1;
		if (this->isPossible(new_x, new_y, robotDimensions, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x > 0 && this->y < map.height-1) {
		int new_x = this->x-1;
		int new_y = this->y+1;
		if (this->isPossible(new_x, new_y, robotDimensions, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < map.width-1 && this->y > 0) {
		int new_x = this->x+1;
		int new_y = this->y-1;
		if (this->isPossible(new_x, new_y, robotDimensions, map)) {
			result.push_back(Node(new_x, new_y));
		}
	}
		
#endif


	return result;
}


bool Node::isPossible(int x, int y, MapData map)
{
	return map.at(x, y) < OCC_THRESHOLD;
}

bool Node::isPossible(int x, int y, Rect robotDimensions, MapData map)
{
	int dx = x-this->x;
	int dy = y-this->y;
	double orientation = atan2(dy, dx);
	
	return this->isPossible(x, y, map) && 
			this->robotFitsAtNode(this->x, this->y, orientation, robotDimensions, map) && 
			this->robotFitsAtNode(x, y, orientation, robotDimensions, map);
}

bool Node::isPossible(Node n, Rect robotDimensions, MapData map)
{
	return this->isPossible(n.x, n.y, robotDimensions, map);
}

bool Node::robotFitsAtNode(int x, int y, double orientation, Rect r, MapData map)
{
	Point bias = CreatePoint(x, y)*map.resolution;
	Point left_front = (rotatePoint(r.left_front, orientation)+bias)/map.resolution;
	Point right_front = (rotatePoint(r.right_front, orientation)+bias)/map.resolution;
	Point left_rear = (rotatePoint(r.left_rear, orientation)+bias)/map.resolution;
	Point right_rear = (rotatePoint(r.right_rear, orientation)+bias)/map.resolution;
	
	//ROS_INFO("Checking if robot fits at %d,%d oriented at %.2f", x, y, orientation);
	
    std::stringstream sstr;
	sstr << "Robot grid dims: " << left_front << ", " << right_front << ", " << left_rear << ", " << right_rear;
	//ROS_INFO("%s", sstr.str().c_str());
	
	int min_x = std::min(std::min(std::min(left_front.x, right_front.x), left_rear.x), right_rear.x);
	int min_y = std::min(std::min(std::min(left_front.y, right_front.y), left_rear.y), right_rear.y);
	int max_x = std::max(std::max(std::max(left_front.x, right_front.x), left_rear.x), right_rear.x);
	int max_y = std::max(std::max(std::max(left_front.y, right_front.y), left_rear.y), right_rear.y);
	
	
	if (min_y != left_front.y && min_y != right_front.y) {
		//Flip vertically
		std::swap(left_front, left_rear);
		std::swap(right_front, right_rear);
	}
	if (min_x != left_front.x && min_x != left_rear.x) {
		//Flip horizontally
		std::swap(left_front, right_front);
		std::swap(left_rear, right_rear);
	}
	Rect rct = CreateRect(left_front, right_front, left_rear, right_rear);
	
	
	for (int i = std::max(0, min_y); i < std::min(max_y, map.height); i++) {
		for (int j = std::max(0, min_x); j < std::min(max_x, map.width); j++) {
			if (map.at(j, i) < OCC_THRESHOLD_2) {
				//Empty area, no worries
				continue;
			}
			
			//ROS_INFO("Checking obstacle %d,%d", j, i);
			
			Point cell_point = CreatePoint(j, i);
			if (in_rectangle(cell_point, rct)) {
				//ROS_ERROR("Will be colliding (occ %d)", map.at(j, i));
				return false;
			}
		}
	}
	
	
	ROS_WARN("Recording waypoint %d,%d", x, y);
	
	return true;
}

Node Node::parent(NodeList parents) {
	Node clone = Node(this->parent_x, this->parent_y);
	NodeList::iterator it = find(parents.begin(), parents.end(), clone);
	if (it != parents.end()) {
		return *it;
	}
	//Return clone of this
	return Node(this->x, this->y);
}

// This function is required for built-in STL list functions like sort
int Node::operator<(const Node &rhs) const
{
	if (this->G+this->H < rhs.G+rhs.F) return 1;
   return 0;
}


void Node::setG(double G)
{
	this->G = G;
	this->has_F = false;
}
void Node::setH(double H)
{
	this->H = H;
	this->has_F = false;
}

double Node::getF()
{
	if (!this->has_F) {
		this->F = this->G+this->H;
		this->has_F = true;
	}
	return this->F;
}

double Node::getG()
{
	return this->G;
}

double Node::getH()
{
	return this->H;
}

void Node::setParent(Node n)
{
	this->parent_x = n.x;
	this->parent_y = n.y;
}

