#ifndef _PLANNING_NODE_H_
#define _PLANNING_NODE_H_

#include "../types.h"

#define OCC_THRESHOLD	50	//Used to separate center nodes
#define OCC_THRESHOLD_2	80	//Used to check C-space orientation affordability

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
	bool isPossible(int x, int y, MapData map);
	bool isPossible(int x, int y, Rect robotDimensions, MapData map);
	bool robotFitsAtNode(int x, int y, double orientation, Rect r, MapData map);
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
	NodeList neighbours(MapData map);
	NodeList neighbours(MapData map, Rect robotDimensions);
	Node parent(NodeList parents);
	bool isPossible(Node n, Rect robotDimensions, MapData map);
	void setG(double G);
	void setH(double H);
	double getF();
	double getG();
	double getH();
	void setParent(Node n);
};

}

#endif //_PLANNING_NODE_H_
