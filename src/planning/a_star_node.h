#ifndef _PLANNING_NODE_H_
#define _PLANNING_NODE_H_

#include "../types.h"

#define OCC_THRESHOLD	80

namespace lunabotics {
	
class Node;
	
typedef std::vector<Node> NodeArr;
typedef std::list<Node> NodeList;
	
class Node {
public:
	int x;
	int y;
	int parent_x;
	int parent_y;
	double F;
	double G;
	double H;
	
	Node();
	Node(const Node &);
	Node(int nx, int ny);
	~Node(){};
	Node &operator=(const Node &rhs);
	int operator==(const Node &rhs) const;
	int operator!=(const Node &rhs) const;
	int operator<(const Node &rhs) const;
	friend std::ostream &operator<<(std::ostream &, const Node &);
	NodeList neighbours(int grid_width, int grid_height, OccupancyArr grid);
	Node parent(NodeList parents);
};

}

#endif //_PLANNING_NODE_H_
