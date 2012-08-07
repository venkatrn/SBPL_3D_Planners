#include <move_base_3D/move_base_3D.h>
#include <sbpl_dynamic_env_global_planner_3D/sbpl_dynamic_env_global_planner_3D.h>

/** Constructor */

MoveBase3D::MoveBase3D()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("static_collision_map", static_collision_map_,std::string(""));
  private_nh.param("local_pub_rate", local_pub_rate_, 1.0);
  private_nh.param("window_side", window_side_, 2.0);

  octomap_server_ = new octomap::OctomapServer(static_collision_map_);  
  SLAM_sub = nh.subscribe("currentPos", 1, &MoveBase3D::SLAMCallback, this);
  local_cmap_pub = nh.advertise<arm_navigation_msgs::CollisionMap>("move_base_3D/local_cmap",1);
  goal_sub = nh.subscribe("goal_pose", 1, &MoveBase3D::goalCallback, this);

  /** This is the place where you need to instantiate your planner's wrapper */
  global_planner = new SBPLDynEnv3DGlobalPlanner();

}


/** Callbacks */

void MoveBase3D::localCMapCallback(const ros::TimerEvent& e)
{
  printf("Publishing Local Collision Map\n");
  arm_navigation_msgs::CollisionMap cmap;
  octomap_server_->getLocalCollisionMap(cmap, currentPos_, window_side_);
  local_cmap_pub.publish(cmap);
}

void MoveBase3D::SLAMCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // pthread_mutex_lock(&m);
  currentPos_ = *msg;
  global_planner->setStart(currentPos_);
  // pthread_mutex_unlock(&m);

}

void MoveBase3D::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
{
  geometry_msgs::PoseStamped goal = *goal_msg;
  arm_navigation_msgs::CollisionMap collision_map;
  octomap_server_->getCollisionMap(collision_map);
  global_planner->makePlan(currentPos_, goal, collision_map);
  return;    
}

/** Public Methods */

void MoveBase3D::getCollisionMap(arm_navigation_msgs::CollisionMap& collision_map)
{
  octomap_server_->getCollisionMap(collision_map);
  return;
}

void MoveBase3D::getCurrentPosition(geometry_msgs::PoseStamped& current_pos)
{
  current_pos = currentPos_;
  return;
}

ros::Timer MoveBase3D::initializeTimer()
{
  ros::Timer local_cmap_timer = nh.createTimer(ros::Duration(1.0/local_pub_rate_),&MoveBase3D::localCMapCallback,this);
  return local_cmap_timer;
}

/** Destructor */

MoveBase3D::~MoveBase3D()
{
  if(octomap_server_ != NULL)
    delete octomap_server_;
  if(global_planner != NULL)
    delete global_planner;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"move_base_3D");
  ros::NodeHandle nh;
  MoveBase3D move_base_3D;
  ros::Timer local_cmap_timer = move_base_3D.initializeTimer();
  ros::spin();
  return 0;
}
