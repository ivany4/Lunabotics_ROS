#include "ros/ros.h"
#include "a_star_node.h"
#include <iostream>
#include <list>
#include <vector>
using namespace std;
using namespace planning;

#define USE_8_DIRECTIONS	1

node::node(): x(0), y(0), parent_x(0), parent_y(0), F(0), G(0), H(0)
{
}

node::node(int nx, int ny): x(nx), y(ny), parent_x(0), parent_y(0), F(0), G(0), H(0)
{
}

node::node(const node &copyin): x(copyin.x), y(copyin.y), parent_x(copyin.parent_x), parent_y(copyin.parent_y), F(copyin.F), G(copyin.G),
								H(copyin.H)
{
}
namespace planning {
std::ostream &operator<<(std::ostream &output, const node &aaa)
{
   output << "(" << aaa.x << "," << aaa.y << ")";// [parent (" << aaa.parent_x << "," << aaa.parent_y << "), F:" << aaa.F << " G:" << aaa.G << " H:" << aaa.H << "]" << endl;
   return output;
}
}

node& node::operator=(const node &rhs)
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

int node::operator==(const node &rhs) const
{
   if( this->x != rhs.x) return 0;
   if( this->y != rhs.y) return 0;

   return 1;
}

int node::operator!=(const node &rhs) const
{
   if( this->x != rhs.x) return 1;
   if( this->y != rhs.y) return 1;

   return 0;
}

node_list node::neighbours(int grid_width, int grid_height, map_grid grid)
{
	node_list result;
	if (this->x > 0) {
		int new_x = this->x-1;
		if (grid.at(new_x+this->y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, this->y));
		}
	}
	if (this->y > 0) {
		int new_y = this->y-1;
		if (grid.at(this->x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(this->x, new_y));
		}
	}
	if (this->x < grid_width-1) {
		int new_x = this->x+1;
		if (grid.at(new_x+this->y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, this->y));
		}
	}
	if (this->y < grid_height-1) {
		int new_y = this->y+1;
		if (grid.at(this->x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(this->x, new_y));
		}
	}
#ifdef USE_8_DIRECTIONS
		
	if (this->x > 0 && this->y > 0) {
		int new_x = this->x-1;
		int new_y = this->y-1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, new_y));
		}
	}
	if (this->x < grid_width-1 && this->y < grid_height-1) {
		int new_x = this->x+1;
		int new_y = this->y+1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, new_y));
		}
	}
	if (this->x > 0 && this->y < grid_height-1) {
		int new_x = this->x-1;
		int new_y = this->y+1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, new_y));
		}
	}
	if (this->x < grid_width-1 && this->y > 0) {
		int new_x = this->x+1;
		int new_y = this->y-1;
		if (grid.at(new_x+new_y*grid_width) < OCC_THRESHOLD) {
			result.push_back(node(new_x, new_y));
		}
	}
		
#endif


	return result;
}

node node::parent(node_list parents) {
	node clone = node(this->parent_x, this->parent_y);
	node_list::iterator it = find(parents.begin(), parents.end(), clone);
	if (it != parents.end()) {
		return *it;
	}
	//Return clone of this
	return node(this->x, this->y);
}

// This function is required for built-in STL list functions like sort
int node::operator<(const node &rhs) const
{
   if (this->F < rhs.F) return 1;
   return 0;
}




