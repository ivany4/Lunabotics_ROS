#include "a_star_node.h"

a_star_node::a_star_node(): x(0), y(0), F(0), G(0), H(0), px(0), py(0), used(false)   // Constructor
{
}

a_star_node::a_star_node(const a_star_node &copyin): x(copyin.x), y(copyin.y), F(copyin.F), G(copyin.G),
								H(copyin.H), px(copyin.px), py(copyin.py), used(copyin.used)
{
}

ostream &operator<<(ostream &output, const a_star_node &aaa)
{
   output << "x:" << aaa.x << " y:" << aaa.y << " F:" << aaa.F << " G:" << aaa.G << " H:" << aaa.H << " parentx:" << aaa.px << " parenty:" << aaa.py << endl;
   return output;
}

a_star_node& a_star_node::operator=(const a_star_node &rhs)
{
   this->x = rhs.x;
   this->y = rhs.y;
   this->F = rhs.F;
   this->G = rhs.G;
   this->H = rhs.H;
   this->px = rhs.px;
   this->py = rhs.py;
   this->used = rhs.used;
   return *this;
}

int a_star_node::operator==(const a_star_node &rhs) const
{
   if( this->x != rhs.x) return 0;
   if( this->y != rhs.y) return 0;
   if( this->F != rhs.F) return 0;
   if( this->G != rhs.G) return 0;
   if( this->H != rhs.H) return 0;
   if( this->px != rhs.px) return 0;
   if( this->py != rhs.py) return 0;
   if( this->used != rhs.used) return 0;

   return 1;
}


// This function is required for built-in STL list functions like sort
/*
    Modifiziert: Sortierung nach F Wert. Aufsteigend.
*/
int a_star_node::operator<(const a_star_node &rhs) const
{
   if( this->F < rhs.F ) return 1;
   return 0;
}



