/** Usage:
  Outdoor Environment (Path to external mesh must be specified in this file): rosrun <package_name> pathVisualizer <path_to_solution_file> <path_to_dyn_obs_file> 
  OR
  Indoor Environment (Path to env file must be passed as command line arg):  rosrun <package_name> pathVisualizer <path_to_solution_file> <path_to_dyn_obs_file>  <path_to_env_file>
 **/

/**
 * Node functionalities:
 * Reads (x,y,z,yaw,time) solution path from a text file and publishes a quadrotor/helicopter mesh as a visualization marker for RVIZ
 * Reads in dynamic obstacle trajectories from a text file and publishes quadrotor/helicopter meshes (visualization markers) for each dynamic obstacle. (Remember to set dynamic obstacle radii under pr2_rad and quad_rad variables)
 * Reads the 3D environment from a text file and publishes a cube list visualization marker array for RVIZ. (Currently supports only environments wherein all static obstacles are on the ground and have the same height)
 *          OR
 * Renders a mesh (collada/stl) (don't forget to specify the mesh URI)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <sbpl/headers.h>
#include <vector>
#include <time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define ENABLE_TIME 1

using namespace std;

char* sol_path, *dyn_obs_path, *env_path;
bool OUTDOOR_ENV;
vector <vector<double> > robotTraj;
vector<vector<vector <double> > > dynObsTrajs;
vector<double> dynObsRad;
const int INF = 10000;

// Env parameters
int mapX,mapY,mapZ;
double spatial_resolution, temporal_resolution;  // Spatial resolution is assumed to be uniform in x,y and z directions.
double center_x,center_y,center_z;

// Dyn Obs parameters
double pr2_rad = 0.6;
double quad_rad = 0.3; 

bool getPoint(FILE* ptr, vector<double>& point){
  if(fscanf(ptr,"%lf %lf %lf %lf %lf",&point[0], &point[1], &point[2], &point[3], &point[4])==EOF)
    return false;
  return true;
}

bool skipLines(FILE* ptr, int n){	// Skips n lines in a file
  char sTemp[1000];
  for(int i=0;i<n;i++){      
    if(fgets(sTemp,1000,ptr)==NULL)
      return false;
  }
  return true;
}


bool renderWorld(ros::NodeHandle node_handle){

  // Read world file (in the format of .exp file) and add cubes at positions where there is '1' in the world file.

  char sTemp[1000];
  int temp;
  FILE* ptr;
  ptr = fopen(env_path,"r");
  if(ptr == NULL){
    SBPL_ERROR("Cannot open env file");
    return 0;
  }
  if(fscanf(ptr,"%s",sTemp)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  
  if(fscanf(ptr,"%d",&mapX)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  
  if(fscanf(ptr,"%d",&mapY)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }    
  if(fscanf(ptr,"%d",&mapZ)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  
  skipLines(ptr,5);
  if(fscanf(ptr,"%s",sTemp)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  
  if(fscanf(ptr,"%lf",&spatial_resolution)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  

  if(fscanf(ptr,"%s",sTemp)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  }  
  if(fscanf(ptr,"%lf",&temporal_resolution)==EOF){
    SBPL_ERROR("Error reading world file");
    return 0;
  } 
  skipLines(ptr,7);

  center_x = mapX*spatial_resolution/2;
  center_y = mapY*spatial_resolution/2;
  center_z = mapZ*spatial_resolution/2;

  ros::Publisher world_pub = node_handle.advertise<visualization_msgs::Marker>("/map/visualization_marker", 10000, true);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p; 
  //for(int z=0;z<mapZ;z++){
  for(int y=0;y<mapY;y++){
    for(int x=0;x<mapX;x++){
      if(fscanf(ptr,"%d",&temp)==EOF){
        SBPL_ERROR("Error reading world file");
        return 0;
      } 
      if(temp==1){
        ROS_INFO("Point at: %d %d\n",x,y);
        p.x = x*spatial_resolution-center_x+spatial_resolution/2;
        p.y = y*spatial_resolution-center_y+spatial_resolution/2;
        p.z = spatial_resolution*(mapZ-5)/2;
        //p.z = z*spatial_resolution;
        marker.points.push_back(p);
      }
    }
    //}
}

marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = spatial_resolution;
marker.scale.y = spatial_resolution;
marker.scale.z = spatial_resolution*mapZ;
marker.color.a = 1.0;
marker.color.r = 0.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
world_pub.publish(marker);
fclose(ptr);
return true;

}

visualization_msgs::Marker renderWorldMesh(ros::NodeHandle node_handle){

  mapX = 500;
  mapY = 500;
  mapZ = 10;
  spatial_resolution = 0.1;

  center_x = mapX*spatial_resolution/2;
  center_y = mapY*spatial_resolution/2;
  center_z = mapZ*spatial_resolution/2;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "world";
  marker.id = 0;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = 10;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 2; //35
  marker.pose.position.y = 2; //-2
  marker.pose.position.z = -4;//-12

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0; //1
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1; //0

  // Set the scale of the marker -- 1x1x1 here means 1m on a side

  marker.scale.x = 0.100;  // This is z in mesh ! 0.150
  marker.scale.y =.065;
  marker.scale.z = .085; //0.075

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.847f;
  marker.color.g = 0.164f;
  marker.color.b = 0.164f;
  marker.color.a = 0.1;

  marker.mesh_resource="package://sbpl_dynamic_planner_3D/config/outdoor_terrain.dae";  // The 3D mesh representing the world
  marker.mesh_use_embedded_materials=1;

  //marker.lifetime = ros::Duration(10000);
  return marker;
}

visualization_msgs::Marker renderPath(ros::NodeHandle node_handle, vector <vector<double> > robotTraj, bool dynObs){


  static int m_id = -1;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = m_id--;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p; 

  for(int i=0;i<(int)robotTraj.size();i++){



    p.x = robotTraj[i][0]-center_x;
    p.y = robotTraj[i][1]-center_y;
    p.z = robotTraj[i][2];
    marker.points.push_back(p);
    ROS_INFO("Line Point at: %lf %lf %lf\n",p.x,p.y,p.z);

  }

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  if(dynObs == false)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }
  else
  {
    marker.color.r = 1.0;
    marker.color.g = 0.95;
    marker.color.b = 0.9;
    marker.color.a = 0.5;
  }

  return marker;

}

bool readRobotTraj(){

  FILE* ptr;
  ptr = fopen(sol_path,"r");
  if(ptr == NULL){
    SBPL_ERROR("Cannot open solution text file");
    return 0;
  }

  vector<double> point;
  point.resize(5);		        //Every point in the trajectory is of the form (x,y,z,yaw,time)
  while(getPoint(ptr,point)){
    robotTraj.push_back(point);
  }

  if(robotTraj.size()==0){ 		//Return true if there is atleast one valid point in the trajectory
    fclose(ptr);
    return false;
  }
  fclose(ptr);
  return true;
}

bool readDynObsTrajs(){
  FILE* ptr;
  ptr = fopen(dyn_obs_path,"r");
  if(ptr == NULL){
    SBPL_ERROR("Cannot open dyn_obs text file");
    return 0;
  }

  //I am assuming that the format of the file complies with that of a dynObs file.
  char sTemp[1000];
  int numDynObs,numPoints;

  vector<double> point;
  point.resize(5);		        //Every point in the trajectory is of the form (x,y,z,time,std_dev)
  if(fscanf(ptr,"%s",sTemp)==EOF){
    SBPL_ERROR("Error reading DynObs file");
    return 0;
  }
  if(fscanf(ptr,"%d",&numDynObs)==EOF){
    SBPL_ERROR("Error reading DynObs file");
    return 0;
  }
  dynObsTrajs.resize(numDynObs);
  dynObsRad.resize(numDynObs);

  for(int j=0;j<numDynObs;j++){
    // Skip the next 5 lines (Stuff not needed here)
    if(skipLines(ptr,2)==0){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    if(fscanf(ptr,"%s",sTemp)==EOF){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    if(fscanf(ptr,"%lf",&dynObsRad[j])==EOF){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    if(skipLines(ptr,4)==0){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    if(fscanf(ptr,"%s",sTemp)==EOF){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    if(fscanf(ptr,"%d",&numPoints)==EOF){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
    //ROS_INFO("Got %d\n",numPoints);
    dynObsTrajs[j].resize(numPoints);  
    for(int i=0;i<numPoints;i++){      
      dynObsTrajs[j][i].resize(5);
      if(getPoint(ptr,point)){
        dynObsTrajs[j][i] = point;

      }
      else{
        SBPL_ERROR("DynObs file has incorrect format");
        return 0;
      }	
    }
    // Skip next line
    if(skipLines(ptr,1)==0){
      SBPL_ERROR("Error reading DynObs file");
      return 0;
    }
  }
  fclose(ptr);
  return true;
}

void interp(vector<double> start,vector<double> end, int interp_factor, int index,vector <double>&newPoint){

  if(interp_factor==0){
    SBPL_ERROR("Must have non-zero interpolation factor\n");
    exit(1);
  }

  for(int i=0;i<5;i++){
    newPoint[i] = start[i]+(end[i]-start[i])*index/interp_factor;
  }

}

bool interpTrajs(vector< vector <double> >& robotTraj, vector<vector<vector <double> > > dynObsTrajs){


  vector< vector <double> > newRobotTraj;
  vector<vector<vector <double> > > newDynObsTrajs;
  int interp_factor = 30;

  newRobotTraj.resize((robotTraj.size()-1)*interp_factor);



  for(int i=0;i<(int)robotTraj.size()-1;i++){
    for(int j=0;j<interp_factor;j++){
      newRobotTraj[i*interp_factor+j].resize(5);
      interp(robotTraj[i],robotTraj[i+1], interp_factor, j,newRobotTraj[i*interp_factor+j]);
    }

  }

  interp_factor = 30;				//10 times the interp factor for robot because time resolution is 10 times lesser for the trajectories here
  newDynObsTrajs.resize(dynObsTrajs.size());
  for(int k=0;k<(int)dynObsTrajs.size();k++){
    for(int i=0;i<(int)dynObsTrajs[k].size()-1;i++){
      newDynObsTrajs[k].resize((dynObsTrajs[k].size()-1)*interp_factor);
      for(int j=0;j<interp_factor;j++){
        newDynObsTrajs[k][i*interp_factor+j].resize(5);
        interp(dynObsTrajs[k][i],dynObsTrajs[k][i+1], interp_factor, j,newDynObsTrajs[k][i*interp_factor+j]);
      }

    }
  }

  robotTraj.clear();
  dynObsTrajs.clear();
  robotTraj = newRobotTraj;
  dynObsTrajs = newDynObsTrajs;
  return true;
}

bool entityToMove(vector<int>& ptrs, int& robot_ptr, int& corresp_obs){

  // corresp_obs is set as the index of the dynamic obstacle that has to make the next move. If the robot has to move next, corresp_obs is set to -1
  // The return value is false only if all trajectories have been executed already.

  double min = (double)INF;
  corresp_obs = INF; 

  if(robot_ptr != -1){
    if(robotTraj[robot_ptr][4]<min){
      min = robotTraj[robot_ptr][4];
      corresp_obs = -1;  
    }  

  }
  else
    return false;

  for(int i=0;i<(int)ptrs.size();i++){
    if(ptrs[i]==-1)			// Obstacle's trajectory has run out if the index is -1
      continue;
    if(dynObsTrajs[i][ptrs[i]][3]<min){
      min = dynObsTrajs[i][ptrs[i]][3];
      corresp_obs = i;
    }
  }

  if(corresp_obs == -1){
    if(robot_ptr+1 < (int)robotTraj.size())
      robot_ptr += 1;         // Increment index if there are more points on the trajectory
    else{
      return false;

    }			

  }
  else if(corresp_obs!=INF){

    if(ptrs[corresp_obs]+1 < (int)dynObsTrajs[corresp_obs].size())
      ptrs[corresp_obs] += 1;         // Increment index if there are more points on the trajectory
    else
      ptrs[corresp_obs] = -1; 	// Set index to -1 when obstacle's trajectory has run out
  }

  return corresp_obs!=INF;	

}


int main(int argc, char **argv){

  if(argc<3)
  {
    SBPL_ERROR("Usage: rosrun <package_name> pathVisualizer <path_to_solution_file> <dyn_obs_path>");
    return 0;
  }
  sol_path = argv[1];
  dyn_obs_path = argv[2];
  if(argc == 4)
  {
    env_path = argv[3];
    OUTDOOR_ENV = 0;
  }
  else
    OUTDOOR_ENV = 1;

  printf("%d\n",argc);

  ros::init(argc, argv, "pathVisualizer");
  ros::NodeHandle n;
  ros::Publisher obs_pub = n.advertise<visualization_msgs::Marker>("/map/visualization_marker", 10000, true);
  ros::Publisher robot_pub = n.advertise<visualization_msgs::Marker>("/base_link/visualization_marker", 10000, true);
  ros::Rate loop_rate(1800); //1800//1200
  tf::TransformBroadcaster odom_broadcaster;

  visualization_msgs::Marker marker; 
  // Render world
  if(OUTDOOR_ENV == 0)
  {
    if(!renderWorld(n))  // For indoor - use renderWorld
      ROS_INFO("Could not render world !\n");
  }  
  else
    marker = renderWorldMesh(n) ; 

  ros::Publisher world_pub = n.advertise<visualization_msgs::Marker>("/map/visualization_marker", 10000, true);

  // Read robot trajectory
  if(!readRobotTraj()){
    SBPL_ERROR("A valid trajectory could not be read from solution text file");
    exit(1);
  }
  int numPoints = robotTraj.size();
  ROS_INFO("Number of points in the robot trajectory: %d\n",numPoints);

  if(!readDynObsTrajs()){
    SBPL_ERROR("There was an error reading the DynObs file");
    exit(1);
  }

  /*
  ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("/map/visualization_marker", 10000, true);
  //Render robot trajectory
  path_pub.publish(renderPath(n,robotTraj,false));

  //Render dynObs trajectories
  for(int i = 0;i<dynObsTrajs.size();i++)
    path_pub.publish(renderPath(n,dynObsTrajs[i],true));
*/
  // Read dynamic obstacle trajectories
  int numDynObs = dynObsTrajs.size();
  ROS_INFO("Number of dynamic obstacles: %d\n",numDynObs);

  //Interpolate Trajetcories
  interpTrajs(robotTraj, dynObsTrajs);

  vector<int> current_indices;
  current_indices.resize(numDynObs);
  int robot_index = 0; 
  int entity_index = INF;		// This holds the index of the entity that has to be moved next 



#if ENABLE_TIME == 1
  double clock_now;
  vector<double> current_times;	
  double robot_current_time = 0;
  current_times.resize(numDynObs);
#endif
  while(entityToMove(current_indices,robot_index,entity_index))
  {

    //world_pub.publish(marker);
    //if(robot_index>1500)
    //	return 0; 

    if(entity_index!=-1&&current_indices[entity_index]==-1)
      continue;         

    //ROS_INFO("Point: %lf %lf %lf\n", robotTraj[robot_index][0],robotTraj[robot_index][1],robotTraj[robot_index][2]);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();

    marker.type = 10;  // Mesh
    marker.action = visualization_msgs::Marker::ADD;

    if(entity_index == -1){

#if ENABLE_TIME == 1
      clock_now = clock();
      while(robotTraj[robot_index][4]*CLOCKS_PER_SEC > clock_now*1.5){		//Wait till sufficient time has elapsed
        clock_now = clock();
      }
      robot_current_time = robotTraj[robot_index][4];
#endif

      btQuaternion temp;
      temp.setEulerZYX(-1.5-robotTraj[robot_index][3],0,0);

      marker.ns = "robot";
      marker.id = 0;
      marker.pose.position.x = robotTraj[robot_index][0]-center_x;

      if(OUTDOOR_ENV==1)
        marker.pose.position.y = 50-robotTraj[robot_index][1]-center_y; //FLIPUD only for outdoor
      else
        marker.pose.position.y = robotTraj[robot_index][1]-center_y; //FLIPUD only for outdoor

      marker.pose.position.z = robotTraj[robot_index][2];
      marker.pose.orientation.x = temp.getX();
      marker.pose.orientation.y = temp.getY();
      marker.pose.orientation.z = temp.getZ();
      marker.pose.orientation.w = temp.getW();
      marker.color.a = 1.0;
      marker.color.r = 0.0;									
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      if(OUTDOOR_ENV == 1)
      {
        marker.mesh_resource = "package://sbpl_dynamic_planner_3D/config/black/black.dae";
        marker.scale.x = .15; //0.35 for quadrotor, 1 for black, .01 for military_copter
        marker.scale.y = .15;
        marker.scale.z = .15;
        marker.mesh_use_embedded_materials = true;
      }
      else
      {
        marker.scale.x = .25; //0.35 for quadrotor, 1 for black, .01 for military_copter
        marker.scale.y = .25;
        marker.scale.z = .25;
        marker.mesh_resource = "package://sbpl_dynamic_planner_3D/config/quadrotor_base.dae";   // Downloaded from hector_quadrotor ros package - http://mirror.umd.edu/roswiki/tu%282d%29darmstadt%282d%29ros%282d%29pkg.html#hector_models
        marker.mesh_use_embedded_materials = false;
      }

      // Broadcast TF
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotTraj[robot_index][3]);

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = marker.pose.position.x;
      odom_trans.transform.translation.y = marker.pose.position.y;
      odom_trans.transform.translation.z = marker.pose.position.z;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      robot_pub.publish( marker );
    }
    else{

#if ENABLE_TIME == 1
      clock_now = clock();
      while(dynObsTrajs[entity_index][current_indices[entity_index]][3]*CLOCKS_PER_SEC > clock_now*1.5){		//Wait till sufficient time has elapsed
        clock_now = clock();
      }
      current_times[entity_index] = dynObsTrajs[entity_index][current_indices[entity_index]][3];
#endif
      marker.ns = entity_index;
      marker.id = 0;
      marker.pose.position.x = dynObsTrajs[entity_index][current_indices[entity_index]][0]-center_x;
      if(OUTDOOR_ENV==1)
        marker.pose.position.y = 50-dynObsTrajs[entity_index][current_indices[entity_index]][1]-center_y;
      else
        marker.pose.position.y = dynObsTrajs[entity_index][current_indices[entity_index]][1]-center_y;

      marker.pose.position.z = dynObsTrajs[entity_index][current_indices[entity_index]][2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 1.0;
      marker.color.r = 0.0;									
      marker.color.g = 0.0;
      marker.color.b = 0.0;


      if(dynObsRad[entity_index]>=pr2_rad){
        if(OUTDOOR_ENV==1)
          marker.mesh_resource = "package://sbpl_dynamic_planner_3D/config/military_copter.stl";
        else
          marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        //marker.mesh_resource = "package://pr2_description/meshes/torso_v0/torso_lift.dae";
        if(OUTDOOR_ENV==1)
        {
          marker.scale.x = .003; //0.35 for quadrotor, .1 for black, .001 for military_copter
          marker.scale.y = .003;
          marker.scale.z = .003;
        }
        else
        {
          marker.scale.x = 1.2;
          marker.scale.y = .4;
          marker.scale.z = .4;	
          marker.mesh_use_embedded_materials = true;						// Set to true so that mesh properties in the .dae file are used
        }

      } 
      else{
        if(OUTDOOR_ENV == 1)
          marker.mesh_resource = "package://sbpl_dynamic_planner_3D/config/military_copter.stl";
        else
          marker.mesh_resource = "package://sbpl_dynamic_planner_3D/config/quadrotor_base.dae";  
        if(OUTDOOR_ENV == 1)
        {
          marker.scale.x = .003; //0.35 for quadrotor, .1 for black, .001 for military_copter
          marker.scale.y = .003;
          marker.scale.z = .003;
        }
        else
        {
          marker.scale.x = .25;
          marker.scale.y = .25;
          marker.scale.z = .25;
          marker.mesh_use_embedded_materials = true;						// Set to true so that mesh properties in the .dae file are used
        }

      }
      obs_pub.publish( marker );

    }
    //ros::spinOnce();
#if ENABLE_TIME == 0
    loop_rate.sleep();
#endif
  }
  return 0;
}
