#include <iostream>
#include <vector>
#include <boost/thread/recursive_mutex.hpp>

/** ROS **/
#include <ros/ros.h>

/** TF **/
#include <tf/tf.h>
#include <tf/transform_listener.h>

/** Collision Map Msg*/
#include <arm_navigation_msgs/CollisionMap.h>

/** Generic SBPL 3D Planner */
#include <sbpl_3d_planner/sbpl_3d_planner.h>

// sbpl headers
#include <sbpl_dynamic_planner_3D/sbpl_dynamicObstacles.h>
#include <sbpl_dynamic_planner_3D/DiscreteSpaceTimeInformation.h>
#include <sbpl_dynamic_planner_3D/DiscreteSpaceTimeIntervalInformation.h>
#include <sbpl_dynamic_planner_3D/weightedAStar.h>
#include <sbpl_dynamic_planner_3D/intervalPlanner.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <dynamic_obs_msgs/DynamicObstacles.h>

class SBPLDynEnv3DGlobalPlanner : public SBPL3DPlanner 
{
public:
  
  /**
   * @brief  Default constructor
   */
  SBPLDynEnv3DGlobalPlanner();


  /** 
    * The actual initialization happens here
    */
  void initialize(std::string);
  
  ros::Timer initializeTimer();

  /** This is the function that will be called by MoveBase3D when it receives a goal callback */
  void makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const arm_navigation_msgs::CollisionMap& cmap);

  virtual ~SBPLDynEnv3DGlobalPlanner();

private:
  ros::NodeHandle nh;
  void updateOccupancyGrid(const arm_navigation_msgs::CollisionMap& cmap);
  void dynamicObstacleCallback(const dynamic_obs_msgs::DynamicObstaclesConstPtr& msg);

  ros::Time getDynamicObstacles();
  void visualizeExpansions();
  unsigned char convertDistToCost(double);

  vector<SBPL_DynamicObstacle_t>* sensor_dynObs;
  vector<SBPL_DynamicObstacle_t>* current_dynObs;
  vector<SBPL_DynamicObstacle_t>* plan_dynObs;
  ros::Time current_dynObs_timestamp;
  ros::Time plan_dynObs_timestamp;
  ros::Subscriber dynObs_sub;


  sbpl_arm_planner::OccupancyGrid* grid_;   /* Occupancy grid that maintains the distance transform of the collision map returned by octomap server*/
   
  bool initialized_;

  IntervalPlanner* planner;
  DiscreteSpaceTimeIntervalInformation* env;
  
  tf::TransformListener tf_;	          /**< for our costmap_2d::Costmap2DROS instance */

  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */
  double decrease_epsilon_; /**< initial epsilon for beginning the anytime search */
  double time_resolution;
  double dyn_obs_pad_costmap_removal;
  double temporal_padding;
  bool remove_dynObs_from_costmap;

  double worldx_, worldy_, worldz_;   /** World dimensions*/
  double originx_, originy_, originz_ ; /** World origin*/
  unsigned int gridx_, gridy_, gridz_;   /** Grid dimensions*/
  double resolution_; /** Resolution in meters */
  double inflation_radius_; /** Inflation radius in meters */
  string global_frame_id_;
  bool use_collision_map_from_sensors_;
  double cost_decay_radius_; /** All cells farther than this distance (in meters) away from an obstacle cell are treated as free cells */
  int cost_possibly_circumscribed_thresh_, cost_inscribed_thresh_;
  int obst_cost_thresh_, dyn_obs_cost_thresh_;

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */


  ros::Publisher plan_pub_;
  ros::Publisher marker_pub;
  ros::Publisher goal_pub;
  geometry_msgs::PoseStamped prevGoal;
  bool pathDone;
  
  std::vector<geometry_msgs::Point> footprint_;

  boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */

  pthread_mutex_t m;
};
