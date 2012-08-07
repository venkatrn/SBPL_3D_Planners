#ifndef SBPL_3D_PLANNER
#define SBPL_3D_PLANNER
#include <arm_navigation_msgs/CollisionMap.h>
#include <geometry_msgs/PoseStamped.h>

class SBPL3DPlanner
{
  public:
    SBPL3DPlanner(){}
    virtual void setStart(const geometry_msgs::PoseStamped& start) {}
    virtual void makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const arm_navigation_msgs::CollisionMap& cmap) = 0;
};
#endif
