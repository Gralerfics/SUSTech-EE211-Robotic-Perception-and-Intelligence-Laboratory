#include "nav2_my_planner/astar_planner.hpp"

#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace nav2_my_planner {

AStarPlanner::AStarPlanner(int xs, int ys) {
	// create cell arrays
	costarr = NULL;
	potarr = NULL;
	pending = NULL;
	gradx = grady = NULL;
	setNavArr(xs, ys);

	// priority buffers
	pb1 = new int[PRIORITYBUFSIZE];
	pb2 = new int[PRIORITYBUFSIZE];
	pb3 = new int[PRIORITYBUFSIZE];

	// goal and start
	goal[0] = goal[1] = 0;
	start[0] = start[1] = 0;

	// path buffers
	npathbuf = npath = 0;
	pathx = pathy = NULL;
	pathStep = 0.5;
}


AStarPlanner::~AStarPlanner()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// set goal, start positions for the nav fn
//

void AStarPlanner::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[AStarPlanner] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void AStarPlanner::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[AStarPlanner] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// Set/Reset map size
//

void AStarPlanner::setNavArr(int xs, int ys) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[AStarPlanner] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];  // cost array, 2d config space
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];  // navigation potential array
  pending = new bool[ns];
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];
  grady = new float[ns];
}

//
// set up cost array, usually from ROS
//

void AStarPlanner::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS-type cost array
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
        // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {  // not a ROS map, just a PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // don't do borders
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

// returning values

float * AStarPlanner::getPathX() {return pathx;}
float * AStarPlanner::getPathY() {return pathy;}
int AStarPlanner::getPathLen() {return npath;}

// inserting onto the priority blocks

#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}


// Set up navigation potential arrays for new propagation

void AStarPlanner::setupAStarPlanner(bool keepit)
{
  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // outer bounds of cost array
  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // priority buffers
  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));

  // push the goal
  int k = goal[0] + goal[1] * nx;
  push_cur(k + 1);
  push_cur(k - 1);
  push_cur(k - nx);
  push_cur(k + nx);

  // find # of obstacle cells 好像没什么用处
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;  // number of cells that are obstacles
    }
  }
  nobs = ntot;
}

struct Node {
    int x, y;
    COSTTYPE g; // 累积成本
    Node *parent;

    Node(int x, int y, COSTTYPE g_cost, Node *p = nullptr) : x(x), y(y), g(g_cost), parent(p) {}

    COSTTYPE f(const Node &goal) const {
        // 欧氏距离作为启发式
        return g + static_cast<COSTTYPE>(std::sqrt(std::pow(goal.x - x, 2) + std::pow(goal.y - y, 2)));
    }
};

int AStarPlanner::calcPath(int maxIterations)
{
    // check path arrays
    if (npathbuf < maxIterations) {
        if (pathx) {delete[] pathx;}
        if (pathy) {delete[] pathy;}
        pathx = new float[maxIterations];
        pathy = new float[maxIterations];
        npathbuf = maxIterations;
    }

    int stc = start[1] * nx + start[0];

    // set up offset
    float dx = 0;
    float dy = 0;
    npath = 0;

    // go for <n> cycles at most
    for (int i = 0; i < n; i++) {
        // check if near goal
        int nearest_point = std::max(
                0,
                std::min(
                        nx * ny - 1, stc + static_cast<int>(round(dx)) +
                                     static_cast<int>(nx * round(dy))));
        if (potarr[nearest_point] < COST_NEUTRAL) {
            pathx[npath] = static_cast<float>(goal[0]);
            pathy[npath] = static_cast<float>(goal[1]);
            return ++npath;  // done!
        }

        if (stc < nx || stc > ns - nx) {  // would be out of bounds
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
            return 0;
        }

        // add to path
        pathx[npath] = stc % nx + dx;
        pathy[npath] = stc / nx + dy;
        npath++;

        bool oscillation_detected = false;
        if (npath > 2 &&
            pathx[npath - 1] == pathx[npath - 3] &&
            pathy[npath - 1] == pathy[npath - 3])
        {
            RCLCPP_DEBUG(
                    rclcpp::get_logger("rclcpp"),
                    "[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        int stcnx = stc + nx;
        int stcpx = stc - nx;

        // check for potentials at eight positions near cell
        if (potarr[stc] >= POT_HIGH ||
            potarr[stc + 1] >= POT_HIGH ||
            potarr[stc - 1] >= POT_HIGH ||
            potarr[stcnx] >= POT_HIGH ||
            potarr[stcnx + 1] >= POT_HIGH ||
            potarr[stcnx - 1] >= POT_HIGH ||
            potarr[stcpx] >= POT_HIGH ||
            potarr[stcpx + 1] >= POT_HIGH ||
            potarr[stcpx - 1] >= POT_HIGH ||
            oscillation_detected)
        {
            RCLCPP_DEBUG(
                    rclcpp::get_logger("rclcpp"),
                    "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);

            // check eight neighbors to find the lowest
            int minc = stc;
            int minp = potarr[stc];
            int sti = stcpx - 1;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti++;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti++;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti = stc - 1;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti = stc + 1;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti = stcnx - 1;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti++;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            sti++;
            if (potarr[sti] < minp) {minp = potarr[sti]; minc = sti;}
            stc = minc;
            dx = 0;
            dy = 0;

            RCLCPP_DEBUG(
                    rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
                    potarr[stc], pathx[npath - 1], pathy[npath - 1]);

            if (potarr[stc] >= POT_HIGH) {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");
                // savemap("AStarPlanner_highpot");
                return 0;
            }
        } else {  // have a good gradient here
            // get grad at four positions near cell
            gradCell(stc);
            gradCell(stc + 1);
            gradCell(stcnx);
            gradCell(stcnx + 1);


            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
            float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
            float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
            float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y

            // check for zero gradient, failed
            if (x == 0.0 && y == 0.0) {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
                return 0;
            }

            // move in the right direction
            float ss = pathStep / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // check for overflow
            if (dx > 1.0) {stc++; dx -= 1.0;}
            if (dx < -1.0) {stc--; dx += 1.0;}
            if (dy > 1.0) {stc += nx; dy -= 1.0;}
            if (dy < -1.0) {stc -= nx; dy += 1.0;}
        }

        //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //      potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
    }

    std::priority_queue<std::pair<COSTTYPE, Node>, std::vector<std::pair<COSTTYPE, Node>>, std::greater<>> openSet;
    std::unordered_map<int, Node> allNodes;

    Node startNode(start[0], start[1], 0);
    Node goalNode(goal[0], goal[1], 0);

    openSet.emplace(0, startNode);
    allNodes[startNode.x + nx * startNode.y] = startNode;

    while (!openSet.empty() && maxIterations > 0) {
        Node current = openSet.top().second;
        openSet.pop();
        maxIterations--;

        if (current.x == goal[0] && current.y == goal[1]) {
            // 路径找到
            npath = 0;
            for (Node *node = &allNodes[current.x + nx * current.y]; node != nullptr; node = node->parent) {
                if (npath < npathbuf) {
                    pathx[npath] = node->x;
                    pathy[npath] = node->y;
                    npath++;
                }
            }
            return npath;
        }

        // 探索邻居节点
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int newX = current.x + dx;
                int newY = current.y + dy;
                if (newX >= 0 && newX < nx && newY >= 0 && newY < ny && costarr[newY * nx + newX] < COST_OBS) {
                    Node neighbor(newX, newY, current.g + 1, &allNodes[current.x + nx * current.y]);
                    if (pending[newY * nx + newX]) continue;
                    pending[newY * nx + newX] = true;
                    openSet.emplace(neighbor.f(goalNode), neighbor);
                    allNodes[newX + nx * newY] = neighbor;
                }
            }
        }
    }

    // 没有找到路径
    npath = 0;

    if(npath == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
    }

    return npath;
}
