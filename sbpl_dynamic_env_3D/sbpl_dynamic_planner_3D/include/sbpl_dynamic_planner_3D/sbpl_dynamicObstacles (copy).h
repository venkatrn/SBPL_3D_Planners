
#ifndef SBPL_DYNAMICOBSTACLES_H
#define SBPL_DYNAMICOBSTACLES_H

#include <vector>

using namespace std;

/**
 * @brief A point in a dynamic obstacle trajectory. It has an x, y, time, and standard deviation.
 */
typedef struct{
  double x;
  double y;
  int t;
  double std_dev;
} SBPL_Traj_Pt_t;

/**
 * @brief A trajectory of a dynamic obstacle. It has a vector of points (a time-parameterized path). It also has a probability (not used) and a flag which indicates whether the planner should treat the obstacle like it exists as a static obstacle (true) or that it disappears (false) after the end of the trajectory (not used, always false).
 */
typedef struct{
  //the probability that the object follows this trajectory
  double prob;
  //the points along the trajectory
  vector<SBPL_Traj_Pt_t> points;
  //does the obstacle still "exist" and sit in its last configuration after
  //its trajectory is finished
  bool existsAfter;
} SBPL_Trajectory_t;

/**
 * @brief A dynamic obstacle. It has a list of trajectories that the obstacle may follow and a radius.
 */
typedef struct{
  //list of the possible trajectories this obstacle may take
  vector<SBPL_Trajectory_t> trajectories;
  //the radius of the dynamic obstacle in cells
  double radius;
} SBPL_DynamicObstacle_t;

typedef struct{
	double x;
	double y;
	double theta;
  double t;
} SBPL_4Dpt_t;

typedef struct SBPL_4DCELL{
	int x;
	int y;
	int theta;
  int t;
	int iteration;
public:
	bool operator == (SBPL_4DCELL cell) {return (x==cell.x && y==cell.y && theta==cell.theta && t==cell.t);}
} SBPL_4Dcell_t;

#endif
