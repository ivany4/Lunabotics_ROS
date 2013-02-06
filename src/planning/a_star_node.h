#include <iostream>


using namespace std;


class a_star_node
{
   friend ostream &operator<<(ostream &, const a_star_node &);

   public:
      int x;
      int y;
      double F; // heuristik F = G+H (bewegungskosten+luftlinie)
      double G;
      double H;
      int px; //eltern
      int py;
      bool used;

      a_star_node();
      a_star_node(const a_star_node &);
      ~a_star_node(){};
      a_star_node &operator=(const a_star_node &rhs);
      int operator==(const a_star_node &rhs) const;
      int operator<(const a_star_node &rhs) const;
};
