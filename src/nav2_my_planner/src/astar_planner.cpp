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


    AStarPlanner::~AStarPlanner() {
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

    void AStarPlanner::setGoal(int *g) {
        goal[0] = g[0];
        goal[1] = g[1];
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[AStarPlanner] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

    void AStarPlanner::setStart(int *g) {
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

    void AStarPlanner::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown) {
        COSTTYPE *cm = costarr;
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

    float *AStarPlanner::getPathX() { return pathx; }

    float *AStarPlanner::getPathY() { return pathy; }

    int AStarPlanner::getPathLen() { return npath; }

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

    void AStarPlanner::setupAStarPlanner(bool keepit) {
        // reset values in propagation arrays
        for (int i = 0; i < ns; i++) {
            potarr[i] = POT_HIGH;
            if (!keepit) {
                costarr[i] = COST_NEUTRAL;
            }
            gradx[i] = grady[i] = 0.0;
        }

        // outer bounds of cost array
        COSTTYPE *pc;
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

    int AStarPlanner::calcPath(int maxIterations) {
        // check path arrays
        if (npathbuf < maxIterations) {
            if (pathx) { delete[] pathx; }
            if (pathy) { delete[] pathy; }
            pathx = new float[maxIterations];
            pathy = new float[maxIterations];
            npathbuf = maxIterations;
        }

        int stc = start[1] * nx + start[0];

        npath = 0;

        std::priority_queue < std::pair < COSTTYPE, Node >, std::vector < std::pair < COSTTYPE, Node >>, std::greater<>>
        openSet;
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
                std::vector <Node> tempPath;
                for (Node *node = &allNodes[current.x + nx * current.y]; node != nullptr; node = node->parent) {
                    tempPath.push_back(*node);
                }

                npath = tempPath.size();
                for (int i = 0; i < npath; ++i) {
                    pathx[i] = tempPath[npath - 1 - i].x; // 反转路径
                    pathy[i] = tempPath[npath - 1 - i].y; // 反转路径
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

        if (npath == 0) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
        }

        return npath;
    }

}

}
