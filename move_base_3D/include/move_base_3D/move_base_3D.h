#include <ros/ros.h>
#include <move_base_3D/OctomapServer.h>   
#include <arm_navigation_msgs/CollisionMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <sbpl_3d_planner/sbpl_3d_planner.h>

class MoveBase3D
{

  public:
    MoveBase3D();
    void getCollisionMap(arm_navigation_msgs::CollisionMap& collision_map);
    void getCurrentPosition(geometry_msgs::PoseStamped& current_pos);
    ros::Timer initializeTimer();
    virtual ~MoveBase3D();

  private:
    ros::NodeHandle nh;
    ros::Publisher local_cmap_pub; /** Publisher for the local collision map */
    ros::Subscriber SLAM_sub; /** Subscriber for updating current position from SLAM */
    ros::Subscriber goal_sub; /** Subscriber for the goal topic */

    /* TODO: Make this a parameter. Like an action lib plugin.*/
    SBPL3DPlanner* global_planner;  /** Instance of the global planner class*/


    void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal); /** Callback for the goal topic*/
    void localCMapCallback(const ros::TimerEvent&);                   /** Timer Event Callback for local collision map publisher */
    void SLAMCallback(const geometry_msgs::PoseStampedConstPtr& msg); /** Callback to update current position from SLAM */
    octomap::OctomapServer* octomap_server_;   /* OctomapServer that incrementally builds the collision map from laser*/

    geometry_msgs::PoseStamped currentPos_; 

    /* ROS Params */
    std::string static_collision_map_;/** Set to non-empty string if loading a pre-built collision map (.bt file)*/
    double local_pub_rate_;   /** Local window collision map publisher rate*/
    double window_side_;  /** Side length of the local collision map cube */
};
