#ifndef NAV2_MY_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_MY_PLANNER__ASTAR_PLANNER_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace nav2_my_planner {

#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254  // 254 for forbidden regions
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles

// AStarPlanner cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 50  // Set this to "open space" value
#define COST_FACTOR 0.8  // Used for translating costs in AStarPlanner::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10  // unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000

/**
  Navigation function call.
  \param costmap Cost map array, of type COSTTYPE; origin is upper left
    NOTE: will be modified to have a border of obstacle costs
  \param nx Width of map in cells
  \param ny Height of map in cells
  \param goal X,Y position of goal cell
  \param start X,Y position of start cell

Returns length of plan if found, and fills an array with x,y interpolated
positions at about 1/2 cell resolution; else returns 0.
*/

struct Node {
	int x, y;
	COSTTYPE g; // Cumulative cost
	Node *parent;

	Node(int x, int y, COSTTYPE g_cost, Node *p = nullptr);
	COSTTYPE f(const Node &goal) const;
};

int create_nav_plan_astar(
  const COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan);

class AStarPlanner {
public:
	AStarPlanner(int nx, int ny);
	~AStarPlanner();

	void setNavArr(int nx, int ny);
	int nx, ny, ns;  /**< size of grid, in pixels */

  	void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  	bool calcAStarPlanner();

	float * getPathX();

	float * getPathY();

	int getPathLen();

	float getLastPathCost();

	/** cell arrays */
	COSTTYPE * costarr;  /**< cost array in 2D configuration space */
	float * potarr;  /**< potential array, navigation function potential */
	bool * pending;  /**< pending cells during propagation */
	int nobs;  /**< number of obstacle cells */

	/** block priority buffers */
	int * pb1, * pb2, * pb3;  /**< storage buffers for priority blocks */
	int * curP, * nextP, * overP;  /**< priority buffer block ptrs */
	int curPe, nextPe, overPe;  /**< end points of arrays */

	/** block priority thresholds */
	float curT;  /**< current threshold */
	float priInc;  /**< priority threshold increment */

	void setGoal(int * goal);
	void setStart(int * start);

	int goal[2];
	int start[2];
	
	void initCost(int k, float v);

	void updateCell(int n);
	void updateCellAstar(int n);

	void setupAStarPlanner(bool keepit = false);
	bool propAStarPlannerAstar(int cycles);  /**< returns true if start point found */

	/** gradient and paths */
	float * gradx, * grady;  /**< gradient arrays, size of potential array */
	float * pathx, * pathy;  /**< path points, as subpixel cell coordinates */
	int npath;  /**< number of path points */
	int npathbuf;  /**< size of pathx, pathy buffers */

	float last_path_cost_;  /**< Holds the cost of the path found the last time A* was called */

<<<<<<< HEAD
	int calcPath(int maxcycles);
=======
	int calcPath(int n);
>>>>>>> d479e50 (bkup.)

	float gradCell(int n);  /**< calculates gradient at cell <n>, returns norm */

	float pathStep;  /**< step size for following gradient */
};

}

#endif