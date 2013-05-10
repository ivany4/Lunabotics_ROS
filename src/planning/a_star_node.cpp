#include "ros/ros.h"
#include "a_star_node.h"
#include <iostream>
#include <list>
#include <vector>
using namespace lunabotics;

#define USE_8_DIRECTIONS	1

Node::Node(): x(0), y(0), parent_x(0), parent_y(0), F(0), G(0), H(0)
{
}

Node::Node(int nx, int ny): x(nx), y(ny), parent_x(0), parent_y(0), F(0), G(0), H(0)
{
}

Node::Node(const Node &copyin): x(copyin.x), y(copyin.y), parent_x(copyin.parent_x), parent_y(copyin.parent_y), F(copyin.F), G(copyin.G),
								H(copyin.H)
{
}
namespace lunabotics {
std::ostream &operator<<(std::ostream &output, const Node &aaa)
{
   output << "(" << aaa.x << "," << aaa.y << ")";// [parent (" << aaa.parent_x << "," << aaa.parent_y << "), F:" << aaa.F << " G:" << aaa.G << " H:" << aaa.H << "]" << endl;
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
   this->parent_x = rhs.parent_x;
   this->parent_y = rhs.parent_y;
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

NodeList Node::neighbours(int grid_width, int grid_height, OccupancyArr grid)
{
	NodeList result;
	if (this->x > 0) {
		int new_x = this->x-1;
		if (grid.at(new_x+this->y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y > 0) {
		int new_y = this->y-1;
		if (grid.at(this->x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(this->x, new_y));
		}
	}
	if (this->x < grid_width-1) {
		int new_x = this->x+1;
		if (grid.at(new_x+this->y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, this->y));
		}
	}
	if (this->y < grid_height-1) {
		int new_y = this->y+1;
		if (grid.at(this->x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(this->x, new_y));
		}
	}
#ifdef USE_8_DIRECTIONS
		
	if (this->x > 0 && this->y > 0) {
		int new_x = this->x-1;
		int new_y = this->y-1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < grid_width-1 && this->y < grid_height-1) {
		int new_x = this->x+1;
		int new_y = this->y+1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x > 0 && this->y < grid_height-1) {
		int new_x = this->x-1;
		int new_y = this->y+1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, new_y));
		}
	}
	if (this->x < grid_width-1 && this->y > 0) {
		int new_x = this->x+1;
		int new_y = this->y-1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(Node(new_x, new_y));
		}
	}
		
#endif


	return result;
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
   if (this->F < rhs.F) return 1;
   return 0;
}




