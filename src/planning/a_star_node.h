#ifndef _PLANNING_NODE_H_
#define _PLANNING_NODE_H_

#include "../types.h"

#define OCC_THRESHOLD	80

namespace lunabotics {
	
class Node;
	
typedef std::vector<Node> NodeArr;
typedef std::list<Node> NodeList;
	
class Node {
private:
	double F;
	double G;
	double H;
	bool has_F;
public:
	int x;
	int y;
	int parent_x;
	int parent_y;
	bool essential;
	
	Node();
	Node(const Node &);
	Node(int nx, int ny);
	Node(Point p);
	~Node(){};
	Node &operator=(const Node &rhs);
	int operator==(const Node &rhs) const;
	int operator!=(const Node &rhs) const;
	int operator<(const Node &rhs) const;
	friend std::ostream &operator<<(std::ostream &, const Node &);
	NodeList neighbours(int grid_width, int grid_height, OccupancyArr grid);
	Node parent(NodeList parents);
	void setG(double G);
	void setH(double H);
	double getF();
	double getG();
	double getH();
};

}

#endif //_PLANNING_NODE_H_
