#include "a_star_node.h"

class a_star_graph
{
private:
	int dif(int w1, int w2);
	bool enter(int start_x, int start_y, int goal_x, int goal_y);
	double prune(int start_x, int start_y, int goal_x, int goal_y);
	int width;
	int height;
	std::vector<int8_t> tmap;
	a_star_node expand(int start_x, int start_y, int x, int y, double cost);
	int goal_x;
	int goal_y;

public:
	a_star_graph();
	int counter;
	std::vector<a_star_node> get_graph(std::vector<int8_t> map, 
								int width, int height, 
								int start_x, int start_y, 
								int goal_x, int goal_y);
};
