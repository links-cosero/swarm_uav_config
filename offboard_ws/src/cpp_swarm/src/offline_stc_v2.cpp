#include <iostream>
#include <fstream>
#include <list>

#define MAP_WIDTH 3
#define MAP_HEIGHT 3
#define CELL_UNIT 1
#define SUBUNIT_CELL 0.5

//To simplify accessing neighbours in these directions
enum {POS_X, NEG_Y, NEG_X, POS_Y};

//To simplify subcell identification
enum {TOP_RIGHT, TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT};

//Holds the data of a single subcell in the Group
struct SubCell
{
  int x;
  int y;

  void display()
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  bool operator==(const SubCell& subcell)
  {
    return (x==subcell.x && y==subcell.y);
  }
};

class SubCellGroup
{
public:
  //Define an array of 4 subcells {TOP_RIGHT< TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT}
  SubCell subcells[4];

  // Given the 2D cell coordinates, constructs the 4 subcells
  SubCellGroup(int parent_x, int parent_y)
  {
    subcells[TOP_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[TOP_RIGHT].y=parent_y-SUBUNIT_CELL/2;

    subcells[TOP_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[TOP_LEFT].y=parent_y-SUBUNIT_CELL/2;

    subcells[BOTTOM_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[BOTTOM_LEFT].y=parent_y+SUBUNIT_CELL/2;

    subcells[BOTTOM_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[BOTTOM_RIGHT].y=parent_y+SUBUNIT_CELL/2;
  }
};

class Cell
{
  int x;
  int y;

  bool visited=false;

public:
  SubCellGroup* child_subcellgroup;

  //array of 4 pointers to Cell to hold neighbours
  Cell* neighbours[4] = {NULL, NULL, NULL, NULL};

  Cell(int x, int y)
  {
    this->x=x;
    this->y=y;
  }

  int getX()
  {
    return x;
  }

  int getY()
  {
    return y;
  }

  void markVisited()
  {
    visited=true;
  }

  bool isVisited()
  {
    return visited;
  }

  void display() const
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
  }

  bool operator==(Cell& cell)
  {
    return (cell.getX()==x && cell.getY()==y);
  }

  //As the cell in the list of all cells is being modified,
  //we must deal in pointers
  void addConnection(std::list<Cell>::iterator neighbour, int dir)
  {
    neighbours[dir]=&(*neighbour);
  }

  //Sets a pointer to corresponding subcell group
  void setSubCellGroup(SubCellGroup* child_subcellgroup)
  {
    this->child_subcellgroup=child_subcellgroup;
  }

};

// Class that is entitled to specify the list of obstacles in the Grid Map
class Map
{
public:
  //This structure specifies the four corners of an Obstacle
  struct Obstacle
  {
    int top_left_column;
    int top_left_row;
    int bottom_right_column;
    int bottom_right_row;

    Obstacle(int top_left_column, int top_left_row,
             int bottom_right_column, int bottom_right_row)
    {
      this->top_left_column=top_left_column;
      this->top_left_row = top_left_row;
      this->bottom_right_column=bottom_right_column;
      this->bottom_right_row = bottom_right_row;
    }
  };

  //A list of 4 element integer arrays that specify corners of each Obstacle.
  std::list<Obstacle> all_obstacles;
  //An iterator over this list tht can be used by other classes
  std::list<Obstacle>::iterator obs_it;

  Map()
  {
    int num_obstacles;
    int tlc, tlr, brc, brr;

    //Write obstacle data to csv file
    std::ofstream csv_file;
    csv_file.open("map.csv");

    std::cout<<"Enter number of obstacles: ";
    std::cin>>num_obstacles;

    for(int i=0; i<num_obstacles; i++)
    {
      std::cout<<"Enter obstacle top left column number: ";
      std::cin>>tlc;
      std::cout<<"Enter obstacle top left row number: ";
      std::cin>>tlr;
      std::cout<<"Enter obstacle bottom right column number: ";
      std::cin>>brc;
      std::cout<<"Enter obstacle bottom right row number: ";
      std::cin>>brr;

      csv_file<<tlc<<","<<tlr<<","<<brc<<","<<brr<<std::endl;

      //push a test Obstacle
      all_obstacles.push_back(Obstacle(tlc, tlr, brc, brr));
    }
  }
};

// Class that create and handles the spanning tree associated with the Grid Map
class STC_handler
{
  //list of all cells
  std::list<Cell> all_cells;

  //list of all pointers cells in order of spanning tree
  std::list<Cell*> spanning_tree_cells;

  //returns a neighbour in the specified direction to the cell
  std::list<Cell>::iterator getNeighbour(Cell curr_cell, int dir)
  {
    int offset_x=0;
    int offset_y=0;

    offset_x=(dir==POS_X)?CELL_UNIT:
      (dir==NEG_X)?-CELL_UNIT:0;

    offset_y=(dir==POS_Y)?CELL_UNIT:
      (dir==NEG_Y)?-CELL_UNIT:0;

    // std::cout << "Curr cell: " << '\n';
    // curr_cell.display();

    // std::cout <<"offset_x: " <<offset_x<<" Offest Y: "<<offset_y<< '\n';

    Cell search_cell(curr_cell.getX()+offset_x, curr_cell.getY()+offset_y);
    // std::cout << "Search cell: " << '\n';
    // search_cell.display();

    for(auto i=all_cells.begin(); i!=all_cells.end(); i++)
    {
      if(*i==search_cell)
        return i;
    }

    return all_cells.end();
  }

  //tells if the cell is unoccupied or not
  bool isUnoccupied(Cell curr_cell, Map main_map)
  {
    //checking if the proposed new node lies in an obsacle
    //How to do it: start from top left corner to bottom right, create list of
    //illegal rows and columns (br[0]-tl[0]->rows)
    for(main_map.obs_it=main_map.all_obstacles.begin();
      main_map.obs_it!=main_map.all_obstacles.end(); main_map.obs_it++)
    {
      if(curr_cell.getX()<=main_map.obs_it->bottom_right_column &&
        curr_cell.getX()>=main_map.obs_it->top_left_column)
        if(curr_cell.getY()<=main_map.obs_it->bottom_right_row &&
          curr_cell.getY()>=main_map.obs_it->top_left_row)
          return false;
    }
    return true;
  }

public:
  //constructor sets up the graph and clears the csv file
  STC_handler(Cell start_cell, Map STC_map)
  {
    //Add all valid cells (the ones with no obstacles) to the list of all cells
    //top to bottom
    for(int i=start_cell.getY(); i<MAP_HEIGHT; i+=CELL_UNIT)
    {
      //left to right
      for(int ii=start_cell.getX(); ii<MAP_WIDTH; ii+=CELL_UNIT)
      {
        //if within map boundaries
        if(i-SUBUNIT_CELL>=0 && i+SUBUNIT_CELL<=MAP_HEIGHT
          && ii-SUBUNIT_CELL>=0 && ii+SUBUNIT_CELL<=MAP_WIDTH
            && isUnoccupied(Cell(ii, i), STC_map))
        {
          //Add to list of all cells
          all_cells.push_back(Cell(ii, i));
        }
      }
    }

    // Cycle through the list of all cells and connect them to neighbours
    // Connects each 2D Cell with all its 2D Cell neighbours 
    for(auto valid_cell=all_cells.begin(); valid_cell!=all_cells.end(); valid_cell++)
    {
      // find the pointer to cell in list that is +x neighbour of this cell
      auto neighbour=getNeighbour(*valid_cell, POS_X);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, POS_X);
        neighbour->addConnection(valid_cell, NEG_X);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      // find the pointer to cell in list that is -y neighbour of this cell
      neighbour=getNeighbour(*valid_cell, NEG_Y);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, NEG_Y);
        neighbour->addConnection(valid_cell, POS_Y);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      // find the pointer to cell in list that is -x neighbour of this cell
      neighbour=getNeighbour(*valid_cell, NEG_X);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, NEG_X);
        neighbour->addConnection(valid_cell, POS_X);
      }
      // else std::cout << "Not found" << '\n';
      // std::cout << "--------" << '\n';

      // find the pointer to cell in list that is +y neighbour of this cell
      neighbour=getNeighbour(*valid_cell, POS_Y);
      if(neighbour!=all_cells.end())
      {
        valid_cell->addConnection(neighbour, POS_Y);
        neighbour->addConnection(valid_cell, NEG_Y);
      }
      // else std::cout << "Not found" << '\n';
      //
      // std::cout << "------------------" << '\n';
    }

    //clear the CSV files
    std::ofstream csv_file;
    csv_file.open("circumnavigate_points.csv");
    csv_file.close();
    csv_file.open("offline_stc_points.csv");
    csv_file.close();
  }

  void showCells()
  {
    for(auto i:all_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i.display();
      std::cout << "NEIGHBOURS: " << '\n';
      for(auto ii:i.neighbours)
      {
        if(ii!=NULL) ii->display();
      }
    }
  }

  void showSpanningTree()
  {
    for(auto i:spanning_tree_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i->display();
      std::cout << "NEIGHBOURS: " << '\n';
      for(auto ii:i->neighbours)
      {
        if(ii!=NULL) ii->display();
      }
    }
  }

  //returns a pointer to the first cell
  Cell* getAllCellsBegin()
  {
    return &(*all_cells.begin());
  }

  //returns an iterator pointing to the first element of spanning tree cells
  std::list<Cell*>::iterator getSpanningTreeCellsBegin()
  {
    return spanning_tree_cells.begin();
  }

  //Writes the current cell to a CSV file
  void writeCellToCSV(Cell* curr_cell)
  {
    std::ofstream csv_file;
    csv_file.open("offline_stc_points.csv", std::ios::app);

    csv_file<<curr_cell->getX()<<","<<curr_cell->getY()<<std::endl;
  }

  //Writes the current subcell to a CSV file
  void writeSubCellToCSV(SubCell* curr_subcell)
  {
    std::ofstream csv_file;
    csv_file.open("circumnavigate_points.csv", std::ios::app);

    csv_file<<curr_subcell->x<<","<<curr_subcell->y<<std::endl;
  }

  /**Recursive function that traverses the graph using DFS and n iteratorupdates 
   * spanning tree cells list. This is done in order to construct a spanning tree, in this case minimum 
   * since all the edges have the same weigth. Another alternative would be PRIM algorithm. As opposed to Kruskal, it allow to choose
   * a starting cell, that is the cell from which the drone will start covering.**/
  void DFS(Cell* prev_cell, Cell* curr_cell)
  {
    // curr_cell->display();
    curr_cell->markVisited();
    spanning_tree_cells.push_back(curr_cell);
    writeCellToCSV(curr_cell);

    // std::cout << "-----------" << '\n';

    for(Cell* neighbour:curr_cell->neighbours)
    {
      if(neighbour!=NULL)
        if(!neighbour->isVisited()) DFS(curr_cell, neighbour);
    }

    //Add the current cell to list again to allow backtracking in dead end cases
    // curr_cell->display();
    spanning_tree_cells.push_back(curr_cell);
    writeCellToCSV(curr_cell);
  }

  //divide all the cells in the graph into subcells
  // Each 2D Cell is assigned to 4 subcells of D size
  void divideIntoSubcells(std::list<Cell*>::iterator spanning_tree_cell)
  {
    if(spanning_tree_cell==spanning_tree_cells.end())
      return;

    // (*spanning_tree_cell)->display();

    //Create a subcellgroup for the current spanning tree cell
    SubCellGroup *temp_scg=new SubCellGroup((*spanning_tree_cell)->getX(),
                              (*spanning_tree_cell)->getY());

    //Set the subcell group in the corresponding spanning tree cell
    (*spanning_tree_cell)->setSubCellGroup(temp_scg);
    divideIntoSubcells(++spanning_tree_cell);
  }

  void showSpanningTreeWithSubCellGroups()
  {
    for(auto i:spanning_tree_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i->display();
      std::cout << "SUBCELLS: " << '\n';
      for(int ii=0; ii<4; ii++)
      {
        i->child_subcellgroup->subcells[ii].display();
      }
    }
  }

  // Recursive function that allows to circumnavigate the spanning tree in counterclock wise direction
  void circumnavigate(Cell* curr_cell, Cell* next_cell, SubCell* curr_subcell,
                      std::list<Cell*>::iterator spanning_tree_cell, int count)
  {

    // Condizione di Terminazione: se il puntatore punta alla fine della lista di celle all'interno dello spanning tree
    if(spanning_tree_cell==spanning_tree_cells.end())
      return;

    writeSubCellToCSV(curr_subcell);

    std::cout << "CURRENT CELL: " << '\n';
    curr_cell->display();
    std::cout << "CURRENT SUBCELL" << '\n';
    curr_subcell->display();
    std::cout << "NEXT CELL: " << '\n';
    next_cell->display();

    // In case of dead end, attempt to go to next cell instead of looping at the end
    // Achieved by incrememnting the iterator
    if(curr_cell==next_cell)
    {
      curr_cell=next_cell;
      next_cell=*(++spanning_tree_cell);
      // return;
    }

    // create variable for each subcell adjacent to the current subcell

    SubCell up_subcell;
    up_subcell.x=curr_subcell->x;
    up_subcell.y=curr_subcell->y-SUBUNIT_CELL;

    SubCell right_subcell;
    right_subcell.x=curr_subcell->x+SUBUNIT_CELL;
    right_subcell.y=curr_subcell->y;

    SubCell down_subcell;
    down_subcell.x=curr_subcell->x;
    down_subcell.y=curr_subcell->y+SUBUNIT_CELL;

    SubCell left_subcell;
    left_subcell.x=curr_subcell->x-SUBUNIT_CELL;
    left_subcell.y=curr_subcell->y;

    // In the current cell, if the current subcell is the top right subcell,
    // try going up to next cell, else right to same
    if(*curr_subcell==curr_cell->child_subcellgroup->subcells[TOP_RIGHT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[BOTTOM_RIGHT]==up_subcell)
      {
        std::cout << "Should go up" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go left" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_LEFT];
      }
    }

    //if top left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[TOP_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_RIGHT]==left_subcell)
      {
        std::cout << "Should go left" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go down" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
      }
    }

    //if bottom left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_LEFT]==down_subcell)
      {
        std::cout << "Should go down" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go right" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
      }
    }

    //if bottom right
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[BOTTOM_LEFT]==right_subcell)
      {
        std::cout << "Should go right" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        std::cout << "Should go up" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
      }
    }

    // circumnavigate(next_cell, *(spanning_tree_cell), curr_subcell, spanning_tree_cell, count+1);
    circumnavigate(curr_cell, next_cell, curr_subcell, spanning_tree_cell, count+1);
  }
};

int main()
{
  Map STC_map;
  // 1. Setups the graph with all the valid 2*D cells and their respective neighbours
  STC_handler handler(Cell(50, 50), STC_map);
  // handler.showCells();

  // 2. Constructs the spanning tree starting from a given cell
  // at the beginning previous cell and current cell are the same thing 
  handler.DFS(handler.getAllCellsBegin(), handler.getAllCellsBegin());
  // handler.showSpanningTree();

  // 3. Divides all spanning tree cells into cells of size D
  handler.divideIntoSubcells(handler.getSpanningTreeCellsBegin());
  handler.showSpanningTreeWithSubCellGroups();

  auto st_cell_iterator=handler.getSpanningTreeCellsBegin();
  auto curr_cell=*st_cell_iterator;
  SubCell* curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
  st_cell_iterator++;
  auto next_cell=*st_cell_iterator;

  // 3. Circumnavigates the Spanning Tree given the beginning cell, the next one in the spanning tree list
  handler.circumnavigate(curr_cell, next_cell, curr_subcell, st_cell_iterator, 0);
}
