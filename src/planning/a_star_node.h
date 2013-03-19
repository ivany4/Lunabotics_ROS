#ifndef _PLANNING_NODE_H_
#define _PLANNING_NODE_H_

#include "../types.h"

#define OCC_THRESHOLD	80

namespace planning
{
	
	class node;
	
	typedef std::vector<planning::node> node_arr;
	typedef std::list<planning::node> node_list;
	
		
	class node
	{
	
	   public:
	      int x;
	      int y;
	      int parent_x;
	      int parent_y;
	      double F;
	      double G;
	      double H;
	
	      node();
	      node(const node &);
	      node(int nx, int ny);
	      ~node(){};
	      node &operator=(const node &rhs);
	      int operator==(const node &rhs) const;
	      int operator!=(const node &rhs) const;
	      int operator<(const node &rhs) const;
		friend std::ostream &operator<<(std::ostream &, const node &);
	      node_list neighbours(int grid_width, int grid_height, map_grid grid);
	      node parent(node_list parents);
	};

}

#endif //_PLANNING_NODE_H_
