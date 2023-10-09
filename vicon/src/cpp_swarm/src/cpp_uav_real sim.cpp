#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <array>
#include <list>
#include <cmath>
#include <eigen3/Eigen/Eigen>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;
using std::placeholders::_1;

/**GRID MAP DEFINITION: 4 by 2 cells of 60 centimeter size and subcells of size 30 centimeters*/
#define MAP_WIDTH 240	//cm
#define MAP_HEIGHT 120	//cm
#define CELL_UNIT 60	//cm
#define SUBUNIT_CELL 30 //cm

//To simplify accessing neighbours cells in these directions
enum {POS_X, NEG_Y, NEG_X, POS_Y};

//To simplify subcells identification
enum {TOP_RIGHT, TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT};

//Holds the data of a single subcell in the Group
struct SubCell
{
  int x;
  int y;

  float x_drone;
  float y_drone;

  void display()
  {
    std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
    std::cout<<"X_drone: "<<x_drone<<", Y_drone: "<<y_drone<<std::endl;
  }

  bool operator==(const SubCell& subcell)
  {
    return (x==subcell.x && y==subcell.y);
  }

  float getSubCell_x_drone()
  {
    return x_drone;
  }

  float getSubCell_y_drone()
  {
    return y_drone;
  }
};

class SubCellGroup
{
public:
  //Define an array of 4 subcells in the order: {TOP_RIGHT, TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT}
  SubCell subcells[4];

  // Given the 2D cell coordinates, constructs the 4 subcells
  SubCellGroup(int parent_x, int parent_y, float parent_x_drone, float parent_y_drone)
  {

    // Sub cell with Grid origin convention
    subcells[TOP_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[TOP_RIGHT].y=parent_y-SUBUNIT_CELL/2;

    subcells[TOP_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[TOP_LEFT].y=parent_y-SUBUNIT_CELL/2;

    subcells[BOTTOM_LEFT].x=parent_x-SUBUNIT_CELL/2;
    subcells[BOTTOM_LEFT].y=parent_y+SUBUNIT_CELL/2;

    subcells[BOTTOM_RIGHT].x=parent_x+SUBUNIT_CELL/2;
    subcells[BOTTOM_RIGHT].y=parent_y+SUBUNIT_CELL/2;

    //Sub cell with PX4 origin convention 

    subcells[TOP_RIGHT].x_drone=parent_x_drone-0.15;
    subcells[TOP_RIGHT].y_drone=parent_y_drone+0.15;

    subcells[TOP_LEFT].x_drone=parent_x_drone-0.15;
    subcells[TOP_LEFT].y_drone=parent_y_drone-0.15;

    subcells[BOTTOM_LEFT].x_drone=parent_x_drone+0.15;
    subcells[BOTTOM_LEFT].y_drone=parent_y_drone-0.15;

    subcells[BOTTOM_RIGHT].x_drone=parent_x_drone+0.15;
    subcells[BOTTOM_RIGHT].y_drone =parent_y_drone+0.15;
  }
};

class Cell
{
  int x,y; 

  float x_drone, y_drone;

  bool visited=false;

public:
  SubCellGroup* child_subcellgroup;

  //array of 4 pointers to Cell to hold neighbours
  Cell* neighbours[4] = {NULL, NULL, NULL, NULL};

  Cell(int x, int y, float x_drone, float y_drone)
  {
    this->x=x;
    this->y=y;
    this->x_drone=x_drone;
    this->y_drone=y_drone;
  }

  int getX()
  {
    return x;
  }

  int getY()
  {
    return y;
  }

  float getX_drone()
  {
    return x_drone;
  }

  float getY_drone()
  {
    return y_drone;
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
    std::cout<<"X_drone: "<<x_drone<<", Y_drone: "<<y_drone<<std::endl;
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

// Class that create and handles the spanning tree associated with the Grid Map
class STC_handler
{
  //list of all cells
  std::list<Cell> all_cells;

  //list of all pointers cells in order of spanning tree
  std::list<Cell*> spanning_tree_cells;
  std::list<SubCell*> trajectory_subcells;

  //returns a neighbour in the specified direction to the cell
  std::list<Cell>::iterator getNeighbour(Cell curr_cell, int dir)
  {
    int offset_x=0;
    int offset_y=0;

    float offset_x_drone=0;
    float offset_y_drone=0;

    offset_x=(dir==POS_X)?CELL_UNIT:
      (dir==NEG_X)?-CELL_UNIT:0;

    offset_y=(dir==POS_Y)?CELL_UNIT:
      (dir==NEG_Y)?-CELL_UNIT:0;

    offset_y_drone=(dir==POS_X)?0.6:
      (dir==NEG_X)?-0.6:0;

    offset_x_drone=(dir==POS_Y)?0.6:
      (dir==NEG_Y)?-0.6:0;


    Cell search_cell(curr_cell.getX()+offset_x, curr_cell.getY()+offset_y,
                      curr_cell.getX_drone()+offset_x_drone, curr_cell.getY_drone()+offset_y_drone);
    
    for(auto i=all_cells.begin(); i!=all_cells.end(); i++)
    {
      if(*i==search_cell)
        return i;
    }

    return all_cells.end();
  }

  public:
  //constructor sets up the graph and clears the csv file
  STC_handler(Cell start_cell)
  {
    //Add all valid cells to the list of all cells
    //top to bottom

    float x_drone = start_cell.getX_drone();
    float y_drone = start_cell.getY_drone();

    for(int i=start_cell.getY(); i<MAP_HEIGHT; i+=CELL_UNIT)
    {
      //left to right
      for(int ii=start_cell.getX(); ii<MAP_WIDTH; ii+=CELL_UNIT)
      {
        //if within map boundaries
        if(i-SUBUNIT_CELL>=0 && i+SUBUNIT_CELL<=MAP_HEIGHT
          && ii-SUBUNIT_CELL>=0 && ii+SUBUNIT_CELL<=MAP_WIDTH)
        {
          //Add to list of all cells
          all_cells.push_back(Cell(ii, i, x_drone, y_drone));
          y_drone += 0.6;
        }
      }
      x_drone += 0.6;
      y_drone = start_cell.getY_drone();
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
      // std::cout << "--------"<< '\n';

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
  }

  void showCells()
  {
    for(auto i:all_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i.display();
      // std::cout << "NEIGHBOURS: " << '\n';
      // for(auto ii:i.neighbours)
      // {
      //   if(ii!=NULL) ii->display();
      // }
    }
  }

  void showSpanningTree()
  {
    for(auto i:spanning_tree_cells)
    {
      std::cout << "CURRENT CELL: " << '\n';
      i->display();
      //std::cout << "NEIGHBOURS: " << '\n';
      // for(auto ii:i->neighbours)
      // {
      //   if(ii!=NULL) ii->display();
      // }
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

  /**Recursive function that traverses the graph using DFS and n iteratorupdates 
   * spanning tree cells list. This is done in order to construct a spanning tree, in this case minimum 
   * since all the edges have the same weigths. Another alternative would be PRIM algorithm. As opposed to Kruskal, it allow to choose
   * a starting cell, that is the cell from which the drone will start covering.**/
  void DFS(Cell* prev_cell, Cell* curr_cell)
  {
    // curr_cell->display();
    curr_cell->markVisited();
    spanning_tree_cells.push_back(curr_cell);

    for(Cell* neighbour:curr_cell->neighbours)
    {
      if(neighbour!=NULL)
        if(!neighbour->isVisited()) DFS(curr_cell, neighbour);
    }

    //Add the current cell to list again to allow backtracking in dead end cases
    // curr_cell->display();
    spanning_tree_cells.push_back(curr_cell);
    //writeCellToCSV(curr_cell);
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
                              (*spanning_tree_cell)->getY(), (*spanning_tree_cell)->getX_drone(), (*spanning_tree_cell)->getY_drone());

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

  // return the list of all subcells that defines the set of point for circumnavigation of the ST
  std::list<SubCell*>::iterator getTrajectorySubcellsBegin()
  {
      return trajectory_subcells.begin();
  }

   std::list<SubCell*> getTrajectorySubcells()
  {
      return trajectory_subcells;
  }

  void showTrajectoryWithSubCells()
  {
    for(auto i:trajectory_subcells)
    {
      std::cout << "CURRENT SUBCELL: " << '\n';
      i->display();
    }

    std::cout << "NUMBER OF SUBCELLS: ";
    printf("%d\n",(int) trajectory_subcells.size());
  }

  // Recursive function that allows to circumnavigate the spanning tree in counterclock wise direction
  void circumnavigate(Cell* curr_cell, Cell* next_cell, SubCell* curr_subcell,
                      std::list<Cell*>::iterator spanning_tree_cell, int count)
  {

    // Condizione di Terminazione: se il puntatore punta alla fine della lista di 
    // celle all'interno dello spanning tree la funzione termina dato che non ritorna niente
    if(spanning_tree_cell==spanning_tree_cells.end())
      return;

    //writeSubCellToCSV(curr_subcell);

    // std::cout << "CURRENT CELL: " << '\n';
    // curr_cell->display();
    // std::cout << "CURRENT SUBCELL" << '\n';
    // curr_subcell->display();
    
    // std::cout << "NEXT CELL: " << '\n';
    // next_cell->display();

    // In case of dead end, attempt to go to next cell instead of looping at the end
    // Achieved by incrememnting the iterator

    trajectory_subcells.push_back(curr_subcell);
    
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
        // std::cout << "Should go up" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        // std::cout << "Should go left" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_LEFT];
      }
    }

    //if top left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[TOP_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_RIGHT]==left_subcell)
      {
        // std::cout << "Should go left" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_RIGHT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        // std::cout << "Should go down" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
      }
    }

    //if bottom left
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_LEFT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[TOP_LEFT]==down_subcell)
      {
        // std::cout << "Should go down" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[TOP_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        // std::cout << "Should go right" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT];
      }
    }

    //if bottom right
    else if(*curr_subcell==curr_cell->child_subcellgroup->subcells[BOTTOM_RIGHT])
    {
      //see if going up is valid
      if(next_cell->child_subcellgroup->subcells[BOTTOM_LEFT]==right_subcell)
      {
        // std::cout << "Should go right" << '\n';
        curr_subcell=&next_cell->child_subcellgroup->subcells[BOTTOM_LEFT];
        curr_cell=next_cell;
        next_cell=*(++spanning_tree_cell);
      }
      else
      {
        // std::cout << "Should go up" << '\n';
        curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
      }
    }

    // circumnavigate(next_cell, *(spanning_tree_cell), curr_subcell, spanning_tree_cell, count+1);
    circumnavigate(curr_cell, next_cell, curr_subcell, spanning_tree_cell, count+1);
  }
};


class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(STC_handler &handler) : Node("offboard_control")
	{

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // rclcpp::QoS pub_qos = rclcpp::QoS(0)
		// .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		// .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
		// .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/drone2/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/drone2/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/drone2/fmu/in/vehicle_command", 10);
    vehicle_status_sub_ = this->create_subscription<VehicleStatus>("/drone2/fmu/out/vehicle_status", qos, std::bind(&OffboardControl::vehicle_status_cb, this, _1));
    vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>("/drone2/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControl::vehicle_odometry_cb, this, _1));

		mission_state = -1;
    // nav_state = 0;
    // arming_state = 0;

    counter = 0;

    current_waypoint = {0.0, 0.0, -2};
    printf("Waypoint %d: ", counter);
    for(int i=0;i<3;i++)
        printf("%f ",current_waypoint[i]);
    printf("\n");
    counter++;

    trajectory_subcells_iterator = handler.getTrajectorySubcellsBegin();
    trajectory_subcells = handler.getTrajectorySubcells();

		timer_mission = this->create_wall_timer(3s, std::bind(&OffboardControl::mission_cb, this));
    timer_offboard = this->create_wall_timer(100ms, std::bind(&OffboardControl::offboard_cb, this));
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_mission;
  rclcpp::TimerBase::SharedPtr timer_offboard;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

  uint8_t nav_state;
  uint8_t arming_state;
  std::array<float,3UL> drone_position;
  int counter;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped


	int64_t mission_state;   // counter to define mission as state machine
  Eigen::Matrix<double,3,1> current_waypoint;
  std::list<SubCell*>::iterator trajectory_subcells_iterator;
  std::list<SubCell*> trajectory_subcells;

  SubCell * subcell;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, int param1 = 0.0, int param2 = 0.0);
  void mission_cb();
  void offboard_cb();
  void vehicle_status_cb(const VehicleStatus::SharedPtr msg);
  void vehicle_odometry_cb(const VehicleOdometry::SharedPtr msg);
	Eigen::Matrix<double,3,1> FLU2FRD_vector_converter(Eigen::Matrix<double,3,1> vector);
  Eigen::Matrix<double,3,1> ENU2NED_vector_converter(Eigen::Matrix<double,3,1> vector);
};

/**
 * @brief Define a mission composed by a set of waypoints for the drone to reach
*/
void OffboardControl::mission_cb()
{

    if(mission_state == -1)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to Offboard");
        /*  ENABLE OFFBOARD MODE
            Python implementation of Offboard Control
            param1 is set to 1 to enable the custom mode.
            param2 is set to 6 to indicate the offboard mode.
            Other options for param2 include:
            param2 = 1 (MANUAL)
            param2 = 2 (ALTCTL)
            param2 = 3 (POSCTL)
            param2 = 4 (AUTO)
            param2 = 5 (ACRO)
            param2 = 6 (OFFBOARD)
            param2 = 7 (STABILIZED)
            param2 = 8 (RATTITUDE)
        */
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1., 6.);
        if(nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
          mission_state = 0;

    }else if (mission_state == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Arming");
		    this->arm();
        if (arming_state == VehicleStatus::ARMING_STATE_ARMED)
          mission_state = 1;

    }else if (mission_state == 1)
    {
      printf("\nDrone position: ");
      for(int i=0;i<3;i++)
          printf("%f ",drone_position[i]);
      printf("\nSetpoint position: ");    
      for(int i=0;i<3;i++)
          printf("%f ",current_waypoint[i]);

      // Reach initial waypoint current_waypoint = {0.0,0.0,0.5}
      // if the drone is below 5 cm range then compute next waypoint 
        if(abs(drone_position[0] - current_waypoint[0]) <= 0.05 &&
            abs(drone_position[1] - current_waypoint[1]) <= 0.05 &&
            abs(drone_position[2] - current_waypoint[2]) <= 0.1) 
        {
            subcell = *trajectory_subcells_iterator;
            Eigen::Matrix<double,3,1> c_w = {subcell->getSubCell_x_drone(), 
            subcell->getSubCell_y_drone(), 2};

            printf("\n Waypoint before conversion %d: ", counter);
            for(int i=0;i<3;i++)
              printf("%f ",c_w[i]);
        
            current_waypoint = ENU2NED_vector_converter(c_w);
        
            printf("\n Waypoint after conversion %d: ", counter);
            for(int i=0;i<3;i++)
              printf("%f ",current_waypoint[i]);
            counter ++;

            trajectory_subcells_iterator++;
        }
        
        if (subcell == trajectory_subcells.back())
        {
          mission_state = 2;
        }
            
             
    }else if (mission_state == 2)
	  { 
      //check if last waypoint is reached
      if(abs(drone_position[0] - current_waypoint[0]) <= 0.05 &&
              abs(drone_position[1] - current_waypoint[1]) <= 0.05 &&
              abs(drone_position[2] - current_waypoint[2]) <= 0.1) 
      {
        printf("\n");
	  	  RCLCPP_INFO(this->get_logger(), "Landing");
	  	  
        current_waypoint[2] = 0.0;

        mission_state = 3;
      }
    }else if (mission_state == 3)
    {
      printf("\nDrone position: ");
      for(int i=0;i<3;i++)
          printf("%f ",drone_position[i]);
      printf("\nSetpoint position: ");    
      for(int i=0;i<3;i++)
          printf("%f ",current_waypoint[i]);

      float result = abs(drone_position[2] - current_waypoint[2]);
      printf("\nDifference = %f", result);
      if(result <= 0.1)
      {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0);
	  	      mission_state = 4;
      }
	  }else if (mission_state == 4)
	  {
	  	RCLCPP_INFO(this->get_logger(), "Mission finished");
	  	timer_offboard->cancel();
	  	timer_mission->cancel();
	  	exit(1);
	  }
}

/**
 * @brief Define a mission composed by a set of waypoints for the drone to reach
*/
void OffboardControl::offboard_cb()
{
	publish_offboard_control_mode();
	publish_trajectory_setpoint();
} 

void OffboardControl::vehicle_status_cb(const VehicleStatus::SharedPtr msg)
{
  nav_state = msg->nav_state;
  arming_state = msg->arming_state;
}

void OffboardControl::vehicle_odometry_cb(const VehicleOdometry::SharedPtr msg)
{
  drone_position = msg->position;
}

/**
* @brief Function that transforms ROS2 ENU coords to NED coords as defined in https://docs.px4.io/main/en/ros/external_position_estimation.html#reference-frames-and-ros
*/
Eigen::Matrix<double,3,1> OffboardControl::FLU2FRD_vector_converter(Eigen::Matrix<double,3,1> vector)
{
	// Rotation around X-axis of 180° Degrees
	Eigen::Matrix<double,3,3> rotX { {1, 0, 0}, 
									{0, cos(M_PI), -sin(M_PI)},
									{0, sin(M_PI), cos(M_PI)} };

  //matrix*column vector
	vector = rotX*vector;

	return vector;
}

Eigen::Matrix<double,3,1> OffboardControl::ENU2NED_vector_converter(Eigen::Matrix<double,3,1> vector)
{
	// Rotation around Z-axis of 90° Degrees
	Eigen::Matrix<double,3,3> rotZ { {cos(M_PI/2), -sin(M_PI/2), 0},
									                {sin(M_PI/2), cos(M_PI/2), 0}, 
									                {0, 0, 1} };

	vector = rotZ*vector;

	// Rotation around X-axis of 180° Degrees
	Eigen::Matrix<double,3,3> rotX { {1, 0, 0}, 
									                  {0, cos(M_PI), -sin(M_PI)},
								                  	{0, sin(M_PI), cos(M_PI)} };

	vector = rotX*vector;

	return vector;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	std::array<float, 3UL> floatWaypoint;
	for (int i = 0; i < 3; ++i) {
        floatWaypoint[i] = static_cast<float>(current_waypoint(i));
        // printf("%f ", current_waypoint[i]);
    }

	msg.position = floatWaypoint;
	// The drone starts with a yaw of pi/2
	msg.yaw = M_PI/2; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, int param1, int param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[])
{
	// 1. Setups the graph with all the valid 2*D cells and their respective neighbours
	// Values for the cell defined in centimeters
  STC_handler handler(Cell(30, 30,-0.3,-0.9));
  // handler.showCells();

  // 2. Constructs the spanning tree starting from a given cell
  // at the beginning previous cell and current cell are the same thing 
  handler.DFS(handler.getAllCellsBegin(), handler.getAllCellsBegin());
  // handler.showSpanningTree();

  // 3. Divides all spanning tree cells into cells of size D
  handler.divideIntoSubcells(handler.getSpanningTreeCellsBegin());
  // handler.showSpanningTreeWithSubCellGroups();

  // 4. Circumnavigates the Spanning Tree given the beginning cell and 
  //the next one in the spanning tree list
  auto st_cell_iterator=handler.getSpanningTreeCellsBegin();
  auto curr_cell=*st_cell_iterator;
  SubCell* curr_subcell=&curr_cell->child_subcellgroup->subcells[TOP_RIGHT];
  st_cell_iterator++;
  auto next_cell=*st_cell_iterator;

  // Store in order the list of subcells for the drone to cover
  handler.circumnavigate(curr_cell, next_cell, curr_subcell, st_cell_iterator, 0);
  // handler.showTrajectoryWithSubCells();

  // 5. Scrivere un ROS2 service che dia al drone di volta in volta il waypoint da raggiungere
  // e una volta raggiunto il drone richiede il prossimo

	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>(handler));

	rclcpp::shutdown();
	return 0;
}