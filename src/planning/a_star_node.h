#define OCC_THRESHOLD	80

using namespace std;


class a_star_node
{
   friend ostream &operator<<(ostream &, const a_star_node &);

   public:
      int x;
      int y;
      int parent_x;
      int parent_y;
      double F;
      double G;
      double H;

      a_star_node();
      a_star_node(const a_star_node &);
      a_star_node(int nx, int ny);
      ~a_star_node(){};
      a_star_node &operator=(const a_star_node &rhs);
      int operator==(const a_star_node &rhs) const;
      int operator!=(const a_star_node &rhs) const;
      int operator<(const a_star_node &rhs) const;
      std::list<a_star_node> neighbours(int grid_width, int grid_height, std::vector<int8_t> grid);
      a_star_node parent(std::list<a_star_node> parents);
};
