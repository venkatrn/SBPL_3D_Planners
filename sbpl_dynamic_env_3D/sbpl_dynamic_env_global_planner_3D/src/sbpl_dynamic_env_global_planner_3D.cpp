/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mike Phillips (Edited by Venkatraman Narayanan)
 *********************************************************************/

#include <sbpl_dynamic_env_global_planner_3D/sbpl_dynamic_env_global_planner_3D.h>
#include <nav_msgs/Path.h>
#include <sbpl_dynamic_planner_3D/envTime.h>
#include <sbpl_dynamic_planner_3D/envDBubble.h>
#include <sbpl_dynamic_planner_3D/envInterval.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;
using namespace ros;

SBPLDynEnv3DGlobalPlanner::SBPLDynEnv3DGlobalPlanner() {
  SBPLDynEnv3DGlobalPlanner::initialize("asipp");
}

void SBPLDynEnv3DGlobalPlanner::dynamicObstacleCallback(const dynamic_obs_msgs::DynamicObstaclesConstPtr& msg){
  //copy data from param into sensor_dynObs
  double obs_time = msg->header.stamp.toSec();
  //printf("stamp=%f\n",msg->header.stamp.toSec());
  sensor_dynObs->clear();
  for(unsigned int i=0; i<msg->dyn_obs.size(); i++){
    SBPL_DynamicObstacle_t o;
    o.radius = msg->dyn_obs[i].radius;
    for(unsigned int j=0; j<msg->dyn_obs[i].trajectories.size(); j++){
      SBPL_Trajectory_t t;
      t.prob = msg->dyn_obs[i].trajectories[j].probability;
      t.existsAfter = msg->dyn_obs[i].trajectories[j].exists_after;
      for(unsigned int k=0; k<msg->dyn_obs[i].trajectories[j].points.size(); k++){
        geometry_msgs::PoseStamped pose_in;
        pose_in.header = msg->dyn_obs[i].trajectories[j].points[k].header;
        pose_in.header.stamp = ros::Time();
        pose_in.pose = msg->dyn_obs[i].trajectories[j].points[k].pose.pose;
        geometry_msgs::PoseStamped pose_out = pose_in;
        /*
           try{
           tf_.transformPose(costmap_ros_->getGlobalFrameID(), pose_in, pose_out);
           }
           catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           }
           */

        SBPL_Traj_Pt_t p;
        p.x = pose_out.pose.position.x - originx_; //msg->dyn_obs[i].trajectories[j].points[k].pose.pose.position.x;
        p.y = pose_out.pose.position.y - originy_; //msg->dyn_obs[i].trajectories[j].points[k].pose.pose.position.y;
        p.z = pose_out.pose.position.z - originz_; //msg->dyn_obs[i].trajectories[j].points[k].pose.pose.position.y;

        //printf("t=%f\n",msg->dynObs[i].trajectories[j].points[k].time);
        p.t = CONTTIME2DISC(msg->dyn_obs[i].trajectories[j].points[k].header.stamp.toSec() - obs_time, time_resolution);
        p.std_dev = sqrt((msg->dyn_obs[i].trajectories[j].points[k].pose.covariance[0] + msg->dyn_obs[i].trajectories[j].points[k].pose.covariance[7])/2.0);
        t.points.push_back(p);
      }
      o.trajectories.push_back(t);
    }
    sensor_dynObs->push_back(o);
  }

  pthread_mutex_lock(&m);
  //swap sensor_dynObs with current_dynObs
  vector<SBPL_DynamicObstacle_t>* temp = current_dynObs;
  current_dynObs = sensor_dynObs;
  sensor_dynObs = temp;
  current_dynObs_timestamp = msg->header.stamp;
  pthread_mutex_unlock(&m);
}

ros::Time SBPLDynEnv3DGlobalPlanner::getDynamicObstacles(){
  pthread_mutex_lock(&m);

  //check that the current_dynObs are actually more recent than plan_dynObs and if so...
  if(current_dynObs_timestamp > plan_dynObs_timestamp){
    //swap plan_dynObs with current_dynObs
    vector<SBPL_DynamicObstacle_t>* temp = plan_dynObs;
    plan_dynObs = current_dynObs;
    current_dynObs = temp;

    //update our timestamp for most recent dynamic obstacles
    plan_dynObs_timestamp = current_dynObs_timestamp;
  }
  ros::Time t = plan_dynObs_timestamp;
  pthread_mutex_unlock(&m);

  ROS_DEBUG("dynObs size = %d\n",(int)plan_dynObs->size());
  env->setDynamicObstacles(*plan_dynObs);
  return t;
}
void SBPLDynEnv3DGlobalPlanner::updateOccupancyGrid(const arm_navigation_msgs::CollisionMap& collision_map)
{   

  if (collision_map.header.frame_id.compare(global_frame_id_) != 0) {
    ROS_WARN("collision_map_occ is in %s not in %s", collision_map.header.frame_id.c_str(), global_frame_id_.c_str());
    ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map.boxes.size()));
  }

  // add collision map msg
  if (use_collision_map_from_sensors_) {
    grid_->updateFromCollisionMap(collision_map);
  }
  // grid_->visualize();                          
} 

void SBPLDynEnv3DGlobalPlanner::initialize(std::string name) 
{
  initialized_ = false;
  if(!initialized_) 
  {
    ros::NodeHandle private_nh("~");

    ROS_DEBUG("Planner is %s", name.c_str());

    private_nh.param("allocated_time", allocated_time_, 0.5);
    private_nh.param("initial_epsilon",initial_epsilon_,5.0);
    private_nh.param("decrease_epsilon",decrease_epsilon_,1.0);
    private_nh.param("primitive_filename",primitive_filename_,std::string("config/heli_time_0.1.mprim"));
    private_nh.param("temporal_padding",temporal_padding,1.5);

    // Dyn Obs Params
    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);


    private_nh.param("remove_dynObs_from_costmap", remove_dynObs_from_costmap, true);
    private_nh.param("dyn_obs_pad_costmap_removal", dyn_obs_pad_costmap_removal, 0.2);
    private_nh.param("inflation_radius",inflation_radius_,0.55);


    // Map Params
    private_nh.param("worldx",worldx_,20.0);
    private_nh.param("worldy",worldy_,20.0);
    private_nh.param("worldz",worldz_,2.0);

    private_nh.param("originx",originx_,-10.0);
    private_nh.param("originy",originy_,-10.0);
    private_nh.param("originz",originz_,0.0);

    private_nh.param("resolution",resolution_,0.1);
    private_nh.param("time_resolution",time_resolution,0.1);
    private_nh.param("global_frame_id",global_frame_id_,std::string("/map"));
    private_nh.param("use_collision_map_from_sensors",use_collision_map_from_sensors_,true);

    // Costmap Params
    private_nh.param("cost_decay_radius", cost_decay_radius_, 0.75); 
    private_nh.param("cost_inscribed_thresh", cost_inscribed_thresh_,85); //0.5 m  
    private_nh.param("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh_, 80); // 0.4 m
    private_nh.param("obst_cost_thresh", obst_cost_thresh_,255); 
    private_nh.param("dyn_obs_cost_thresh", dyn_obs_cost_thresh_,255); 

    gridx_ = int(worldx_/resolution_+0.5);
    gridy_ = int(worldy_/resolution_+0.5);
    gridz_ = int(worldz_/resolution_+0.5);

    //TODO: Get footprint from somewhere !

    /** Initialize Occupancy Grid */ 
    grid_ = new sbpl_arm_planner::OccupancyGrid(worldx_, worldy_, worldz_, resolution_, originx_, originy_, originz_);
    grid_->setReferenceFrame(global_frame_id_);


  //  octomap_server_ = new octomap::OctomapServer(static_collision_map_);
    std::vector<geometry_msgs::Point> footprint;
    geometry_msgs::Point pt;
    pt.x = 0; pt.y = 0; pt.z = 0.5;
    footprint.push_back(pt);
    pt.x = 0; pt.y = 0; pt.z = -0.5;
    footprint.push_back(pt);
    pt.x = 0.5; pt.y = 0; pt.z = 0;
    footprint.push_back(pt);
    pt.x = -0.5; pt.y = 0; pt.z = 0;
    footprint.push_back(pt);

    env = new EnvIntervalLat();


    env->SetEnvParameter("cost_inscribed_thresh",(unsigned char)cost_inscribed_thresh_);
    env->SetEnvParameter("cost_possibly_circumscribed_thresh",(unsigned char)cost_possibly_circumscribed_thresh_);

    vector<sbpl_3Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_3Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      pt.z = footprint[ii].z;
      perimeterptsV.push_back(pt);
    }
    vector<SBPL_DynamicObstacle_t> init_dynObs;
    env->InitializeEnv(gridx_, // width
        gridy_, // height
        gridz_, // depth 
        0, // mapdata
        0, 0, 0, 0, 0, // start (x, y, z, theta, t)
        0, 0, 0, 0, // goal (x, y, z, theta)
        0, 0, 0, 0,//goal tolerance
        perimeterptsV, resolution_, time_resolution, temporal_padding, 
        nominalvel_mpersecs, timetoturn45degsinplace_secs, (unsigned char)obst_cost_thresh_, (unsigned char)dyn_obs_cost_thresh_,
        primitive_filename_.c_str(), init_dynObs);

    for (ssize_t ix(0); ix < gridx_; ++ix)
      for (ssize_t iy(0); iy < gridy_; ++iy)
        for (ssize_t iz(0); iz < gridz_; ++iz)
        {
          int cell[] = {ix,iy,iz};
          double dist_to_obs = grid_->getCell(cell);
          env->UpdateCost(ix, iy, iz, convertDistToCost(dist_to_obs));
        }

    planner = new IntervalPlanner(env);

    ROS_INFO("[sbpl_dynamic_env_global_planner] Initialized successfully");
    plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

    pthread_mutex_init(&m, NULL);
    sensor_dynObs = new vector<SBPL_DynamicObstacle_t>;
    current_dynObs = new vector<SBPL_DynamicObstacle_t>;
    plan_dynObs = new vector<SBPL_DynamicObstacle_t>;
    current_dynObs_timestamp = ros::Time::now();
    plan_dynObs_timestamp = ros::Time::now(); 
    dynObs_sub = nh.subscribe("dynamic_obstacles", 1, &SBPLDynEnv3DGlobalPlanner::dynamicObstacleCallback, this);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("sbpl_dyn_env_3D/goal",1);
    printf("Constructor Done !\n");
  }
}
/** Destructor */
SBPLDynEnv3DGlobalPlanner::~SBPLDynEnv3DGlobalPlanner()
{
  if(grid_ != NULL)
    delete grid_;
}
unsigned char SBPLDynEnv3DGlobalPlanner::convertDistToCost(double dist)
{
  if(dist>cost_decay_radius_)
    return 0;
  else
    return (unsigned char)(255*(1 - dist/cost_decay_radius_));
}

/** Planner Callback */
void SBPLDynEnv3DGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const arm_navigation_msgs::CollisionMap& cmap) 
{

  /** Update Occupancy Grid using the latest collision map */
  updateOccupancyGrid(cmap);

  vector<geometry_msgs::PoseStamped> plan;
  ROS_INFO("\n\n\n\nstart wrapper\n");
  //clearing plan to make sure that there's no garbage
  plan.clear();

  /*
     ROS_DEBUG("[sbpl_dynamic_env_global_planner] getting fresh copy of costmap");
     costmap_ros_->clearRobotFootprint();
     ROS_DEBUG("[sbpl_dynamic_env_global_planner] robot footprint cleared");

     costmap_ros_->getCostmapCopy(cost_map_);
     */

  ROS_DEBUG("[sbpl_dynamic_env_global_planner] getting dynamic obstacles");
  ros::Time start_time_offset = getDynamicObstacles();
  ROS_DEBUG("[sbpl_dynamic_env_global_planner] done getting dynamic obstacles");

  ROS_INFO("[sbpl_dynamic_env_global_planner] getting start point (%g,%g) goal point (%g,%g)",
      start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  int ret=env->SetStart(start.pose.position.x - originx_, start.pose.position.y - originy_, start.pose.position.z - originz_, theta_start, (start.header.stamp - start_time_offset).toSec());
  if(ret<0 || planner->set_start(ret) == 0){
    ROS_ERROR("ERROR: failed to set start state\n");
    return;
  }

  ret = env->SetGoal(goal.pose.position.x - originx_, goal.pose.position.y - originy_, goal.pose.position.z - originz_, theta_goal);
  if(ret<0 || planner->set_goal(ret) == 0){
    ROS_ERROR("ERROR: failed to set goal state\n");
    return;
  }

  for(unsigned int ix = 0; ix < gridx_; ix++) {
    for(unsigned int iy = 0; iy < gridy_; iy++) {
      for(unsigned int iz = 0; iz < gridz_; iz++) {

        //  unsigned char oldCost = env->GetMapCost(ix,iy,iz);
        //  unsigned char newCost = cost_map_.getCost(ix,iy);

        //TODO: make sure this being commented out
        //doesn't slow things down too much!!!
        //if(oldCost == newCost) continue;

        if(remove_dynObs_from_costmap){
          //start hack to not include dynamic obstacles in cost map
          bool isDynObs = false;
          double res = resolution_;
          for(unsigned int i=0; i < plan_dynObs->size(); i++){
            if(plan_dynObs->at(i).trajectories[0].points.empty())
              continue;
            double dx = plan_dynObs->at(i).trajectories[0].points[0].x - ix*res;
            double dy = plan_dynObs->at(i).trajectories[0].points[0].y - iy*res;
            double dz = plan_dynObs->at(i).trajectories[0].points[0].z - iz*res;
            double d = sqrt(dx*dx+dy*dy+dz*dz);
            if(d < plan_dynObs->at(i).radius + inflation_radius_ + dyn_obs_pad_costmap_removal){
              isDynObs = true;
              break;
            }
          }
          if(isDynObs){
            //sketchy assumption
            //if the dynamic obstacle can stand here then
            //so can the robot (once the dynamic obstacle has
            //move away)
            env->UpdateCost(ix, iy, iz, 0);
            continue;
          }
          //end hack to not include dynamic obstacles in cost map
        }

        int cell[] = {ix,iy,iz};
        double dist_to_obs = grid_->getCell(cell);
        //  printf("%d %d %d: %f\n",ix,iy,iz,dist_to_obs);
        env->UpdateCost(ix, iy, iz, convertDistToCost(dist_to_obs));
      }
    }
  }
  printf("Begin Planning\n");
  planner->force_planning_from_scratch();

  //setting planner parameters
  ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner->set_initialsolution_eps(initial_epsilon_);
  planner->set_decrease_eps_step(decrease_epsilon_);
  planner->set_search_mode(false);

  vector<int> solution_stateIDs;
  ROS_DEBUG("start planner\n");
  if(planner->replan(allocated_time_, &solution_stateIDs))
    ROS_DEBUG("Solution is found");
  else{
    ROS_DEBUG("Solution not found");
    // return false;
  }
  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());
  printf("End Planning\n");

  vector<SBPL_4Dpt_t> sbpl_path;
  env->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.set_poses_size(sbpl_path.size());
  gui_path.header.frame_id = global_frame_id_;
  gui_path.header.stamp = plan_time;
  visualization_msgs::MarkerArray ma;
  int waitCount = 0;
  pathDone = true;
  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_id_;

    pose.pose.position.x = sbpl_path[i].x + originx_;
    pose.pose.position.y = sbpl_path[i].y + originy_;
    pose.pose.position.z = sbpl_path[i].z + originz_; 

    btQuaternion temp;
    temp.setEulerZYX(sbpl_path[i].theta,0,0);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    //printf("%f %f %f %f %f\n",sbpl_path[i].x,sbpl_path[i].y,sbpl_path[i].z,sbpl_path[i].theta,sbpl_path[i].t);

    if(i>0 && pathDone &&
        sbpl_path[i].x == sbpl_path[i-1].x &&
        sbpl_path[i].y == sbpl_path[i-1].y &&
        sbpl_path[i].z == sbpl_path[i-1].z &&
        sbpl_path[i].theta == sbpl_path[i-1].theta){
      ROS_DEBUG("we have a wait!\n");
      pathDone = false;
      prevGoal.header.frame_id = goal.header.frame_id;
      prevGoal.pose.position.x = goal.pose.position.x;
      prevGoal.pose.position.y = goal.pose.position.y;
      prevGoal.pose.position.z = goal.pose.position.z;
      prevGoal.pose.orientation.x = goal.pose.orientation.x;
      prevGoal.pose.orientation.y = goal.pose.orientation.y;
      prevGoal.pose.orientation.z = goal.pose.orientation.z;
      prevGoal.pose.orientation.w = goal.pose.orientation.w;

      visualization_msgs::Marker m;
      m.header.frame_id = global_frame_id_;
      m.header.stamp = ros::Time::now();
      m.ns = "waits";
      m.id = waitCount;
      waitCount++;
      m.text = "WAIT";
      m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = pose.pose.position.x;
      m.pose.position.y = pose.pose.position.y;
      m.pose.position.z = pose.pose.position.z;
      //   m.pose.position.z = 0.01; //p[i].t; 

      m.scale.z = 0.5;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;
      m.lifetime = ros::Duration();

      ma.markers.push_back(m);

      //break;
    }

    gui_path.poses[i].pose.position.x = pose.pose.position.x;
    gui_path.poses[i].pose.position.y = pose.pose.position.y;
    gui_path.poses[i].pose.position.z = pose.pose.position.z;

    if(pathDone)
      plan.push_back(pose);
  }
  /*
     gui_path.set_poses_size(plan.size());
     for(unsigned i=0; i<plan.size(); i++){
     gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
     gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
     gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
     }
     */

  plan_pub_.publish(gui_path);
  marker_pub.publish(ma);
  visualizeExpansions();

  ROS_DEBUG("done wrapper\n\n\n\n");
  return;
}

void SBPLDynEnv3DGlobalPlanner::visualizeExpansions(){
  vector<SBPL_4Dpt_t> p;
  env->getExpansions(&p);
  ROS_DEBUG("size=%d\n",(int)p.size());

  visualization_msgs::MarkerArray ma;
  for(int i=0; i<(int)p.size(); i++){
    visualization_msgs::Marker m;
    m.header.frame_id = global_frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "expands";
    m.id = i;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p[i].x + originx_;
    m.pose.position.y = p[i].y + originy_;
    m.pose.position.z = p[i].z + originz_;
    // m.pose.position.z = 0.001; //p[i].t;

    btQuaternion temp;
    temp.setEulerZYX(p[i].theta,0,0);
    m.pose.orientation.x = temp.getX();
    m.pose.orientation.y = temp.getY();
    m.pose.orientation.z = temp.getZ();
    m.pose.orientation.w = temp.getW();

    m.scale.x = 0.025;
    m.scale.y = 0.025;
    m.scale.z = 0.025;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.5;
    m.lifetime = ros::Duration();

    ma.markers.push_back(m);
  }
  marker_pub.publish(ma);
}
