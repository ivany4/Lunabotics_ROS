#include "ros/ros.h"
#include "a_star_graph.h"
#include <math.h>
#include <list>

#define OCC_THRESHOLD	0.8

using namespace std;

vector<a_star_node> a_star_graph::get_graph(vector<int8_t> map, int width, int height, int start_x, int start_y, int goal_x, int goal_y)
{
	vector<a_star_node> graph;
	
    if (map.at(goal_x+goal_y*width) > OCC_THRESHOLD) { ROS_ERROR("Goal cell is occupied"); return graph; }
    if (map.at(start_x+start_y*width) > OCC_THRESHOLD) { ROS_ERROR("Start cell is occupied"); return graph; }
    
    this->width = width;
    this->height = height;
    this->goal_x = goal_x;
    this->goal_y = goal_y;
    
    counter = 0;

	list<a_star_node> act;
	list<a_star_node> pass;

	for (int i = 0; i < width*height; i++) {
		this->tmap.push_back(map.at(i));
	}

    list<a_star_node>::iterator ci;

    a_star_node A;
    A.x = start_x;
    A.y = start_y;
    A.F = prune(start_x, start_y, goal_x, goal_x) + tmap.at(goal_x+goal_y*width);
    A.G = tmap.at(start_x+start_y*width);
    A.H = prune(start_x, start_y, goal_x, goal_x);
    A.px = start_x;
    A.py = start_y;
    act.push_back(A);

    bool found = false;

    while(!found && !act.empty()) {
        if(counter!=0) {
            act.sort();
            A = act.front();
            start_x = A.x;
            start_y = A.y;
        }

        a_star_node B;
        B = expand(start_x, start_y, start_x+1, start_y, A.G);
        if (B.used) act.push_back(B);

        a_star_node C;
        C = expand(start_x, start_y, start_x-1, start_y, A.G);
        if (C.used) act.push_back(C);

        a_star_node D;
        D = expand(start_x,start_y,start_x,start_y+1, A.G);
        if (D.used) act.push_back(D);

        a_star_node F;
        F = expand(start_x,start_y,start_x,start_y-1, A.G);
        if (F.used) act.push_back(F);

        pass.push_back(A);
        act.remove(A);
        tmap.at(start_x+start_y*width) = 1;

        //if(counter>30)break;

        if (A.x == goal_x && A.y == goal_y) {
			found = true;
		}

        counter++;
    }

    if (!found) ROS_ERROR("Unable to find a route");

    list<a_star_node> to_go;
    bool way_ok = false;
    while (!way_ok && found) {
        a_star_node AA;
        for(ci = pass.begin(); ci != pass.end(); ci++) {
            AA = *ci;
            //cout << A << start_x <<endl;
            if (AA.x == start_x && AA.y == start_y) {
                to_go.push_front(AA);
                start_x = AA.px;
                start_y = AA.py;
                break;
            }

        }
        //Anfang gefunden. Knoten verweist auf sich selbst.
        if (start_x == AA.x && start_y == AA.y) {
			 way_ok = true;  
		}
    }

    if(found) {
        for(ci = to_go.begin(); ci != to_go.end(); ci++) {
            graph.push_back(*ci);
        }
    }

    return graph;
}


double a_star_graph::prune(int sx, int sy, int gx, int gy) {
    if (sx >= 0 && sx < this->width && 
       gx >= 0 && gx < this->width && 
       sy >=0 && sy < this->height && 
       gy >= 0 && gy < this->height) {
        double a;
        double b;
        a = (double)sx - (double)gx;
        b = (double)sy - (double)gy;
        return (int(sqrt(a*a + b*b)));
    }
    else return width*height;
}



bool a_star_graph::enter(int sx, int sy, int gx, int gy) {
    if (sx >= 0 && sx < this->width && 
        gx >= 0 && gx < this->width && 
        sy >= 0 && sy < this->height && 
        gy >= 0 && gy < this->height) {
        if (tmap.at(gx+gy*this->width) < OCC_THRESHOLD && tmap.at(sx+sy*this->width) < OCC_THRESHOLD) {
            if (dif(sx, gx) >= OCC_THRESHOLD && dif(sy, gy) >= OCC_THRESHOLD) {
                  return true;
            }
        }
    }
	return false;
}

int a_star_graph::dif(int w1, int w2) {
	int temp = w1 - w2;
	if (temp < 0) temp *= -1;
	return temp;
}

a_star_node a_star_graph::expand(int sx, int sy, int x, int y, double cost)
{
    a_star_node B;
    if (enter(sx, sy, x, y)) {
        B.x = x;
        B.y = y;
        B.G = tmap[x+(y)*this->width]+cost*0.5;
        B.H = prune(x, y, this->goal_x, this->goal_y);
        B.F = B.H + B.G;
        B.px = sx;
        B.py = sy;
        B.used = true;


/*
        for(ci=act.begin(); ci!=act.end(); ci++)
        {
            a_star_node x = *ci;
            if(x==B)
            {
                cout << "found\n";
                if(x.G<B.G)
                {
                    //cout << "found better\n";
                    act.remove(x);
                    x.px = sx;
                    x.py = sy;
                    x.G = tmap[x.x+x.y*w];
                    x.H = prune(x.x,x.y,gx,gy);
                    x.F = x.G+x.H;
                    act.push_back(x);
                }

                break;
            }
        }
*/


    }
    return B;
}

a_star_graph::a_star_graph(): width(), height(), tmap(), goal_x(), goal_y(), counter()
{
}
