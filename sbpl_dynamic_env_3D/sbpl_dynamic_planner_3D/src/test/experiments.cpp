/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iostream>
using namespace std;

#include <sbpl/headers.h>
#include <sbpl_dynamic_planner_3D/envDBubble.h>
#include <sbpl_dynamic_planner_3D/envTime.h>
#include <sbpl_dynamic_planner_3D/envInterval.h>
#include <sbpl_dynamic_planner_3D/sipp.h>
#include <sbpl_dynamic_planner_3D/weightedAStar.h>
#include <sbpl_dynamic_planner_3D/intervalPlanner.h>

#define PRINT_SPREAD_STATS 0
#define TEMPORAL_PADDING 1

double halfwidth = 0;//0.025;
double halflength = 0;//0.025;
double allocated_time_secs = 300.0; //in seconds
bool bsearchuntilfirstsolution = true;
double initialEps = 100.0;
double dec_eps = 1.0;
bool bforwardsearch = true;
double timeRes = 0.1;

int getNumDynObsCollisions(vector<SBPL_DynamicObstacle_t> dynObs, double px, double py, int pt){
  int collisions = 0;
  for(unsigned int i=0; i < dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs = dynObs.at(i);
    for(unsigned int j=0; j < obs.trajectories.size(); j++){
      SBPL_Trajectory_t traj = obs.trajectories.at(j);
      unsigned int k = 0;
      for(; k < traj.points.size(); k++){
        if(traj.points.at(k).t > pt)
          break;
      }

      if(k==0){
        //This trajectory hasn't even started yet because all of the points in the come after t
        //we assume since we were specifically given a trajectory that starts after the present
        //that the obstacle is irrelevant and for all practical purposes "doesn't exist yet"
        continue;
      }
      else if(k == traj.points.size()){
        //This dynamic obstacle has already finished its trajectory before time t.
        //Here we check if the user wanted to assume the obstacle remains at rest
        //in its final configuration or if it "doesn't exist" after its trajectory.
        if(!traj.existsAfter)
          continue;

        SBPL_Traj_Pt_t p = traj.points.at(k-1);
        double dist = (p.x-px)*(p.x-px) + (p.y-py)*(p.y-py);
        if(sqrt(dist) <= obs.radius + 3*p.std_dev){
          printf("collide rad=%f from k=%d at (%d %d %d)\n", obs.radius, k, CONTXY2DISC(traj.points.at(k).x, 0.025), CONTXY2DISC(traj.points.at(k).y, 0.025), traj.points.at(k).t);
          collisions++;
        }
      }
      else{
        //p1 and p2 are the points before and after cell (in terms of time) respectively
        bool hit = false;
        SBPL_Traj_Pt_t p1, p2;
        p1 = traj.points.at(k-1);
        double dist = (p1.x-px)*(p1.x-px) + (p1.y-py)*(p1.y-py);
        if(abs(traj.points.at(k-1).t-pt) <= TEMPORAL_PADDING && sqrt(dist) <= obs.radius + 3*p1.std_dev)
          hit = true;
        p2 = traj.points.at(k);
        dist = (p2.x-px)*(p2.x-px) + (p2.y-py)*(p2.y-py);
        if(abs(traj.points.at(k).t-pt) <= TEMPORAL_PADDING && sqrt(dist) <= obs.radius + 3*p2.std_dev)
          hit = true;

        if(hit){
          printf("collide rad=%f from k=%d at (%d %d %d), robot at (%d %d %d)\n", 
                 obs.radius, k, 
                 CONTXY2DISC(traj.points.at(k).x, 0.025), 
                 CONTXY2DISC(traj.points.at(k).y, 0.025), 
                 traj.points.at(k).t,
                 CONTXY2DISC(px, 0.025), 
                 CONTXY2DISC(py, 0.025), 
                 pt);
          collisions++;
        }
      }
      
    }
  }
  return collisions;
}


vector<SBPL_DynamicObstacle_t> ReadDynamicObstacles(char* dynObs_filename){
  FILE* fDynObs = fopen(dynObs_filename, "r");
	char sTemp[1024], sTemp1[1024];
  int iTemp;
  vector<SBPL_DynamicObstacle_t> dynamicObstacles;

  //printf("Reading Dynamic Obstacles...\n");

  //get the number of dynamic obstacles in the file
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    printf("ERROR: ran out of dynamic obstacle file early\n");
    exit(1);
  }
	strcpy(sTemp1, "NumberOfDynamicObstacles:");
	if(strcmp(sTemp1, sTemp) != 0){
		printf("ERROR: dynamic obstacle file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    printf("ERROR: ran out of dynamic obstacle file early\n");
    exit(1);
  }
	int numObs = atoi(sTemp);

  //for each dynamic obstacle
  for(int i=0; i < numObs; i++){

    //check that the ID matches i
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "DynamicObstacleID:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    iTemp = atoi(sTemp);
    if(iTemp != i){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %d got %d\n", i, iTemp);
      exit(1);
    }
    SBPL_DynamicObstacle_t obs;

    //Read in the obstacle's radius
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "ObstacleRadius:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    //obs.radiusSquared = atof(sTemp) + EnvNAVXYTHETATIMELATCfg.robotRadius;
    obs.radius = atof(sTemp) + sqrt(halfwidth*halfwidth + halflength*halflength);

    //read the number of trajectories for this obstacle
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "NumberOfTrajectories:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    int numTraj = atoi(sTemp);

    //for each trajectory
    double trajProbSum = 0;
    for(int j=0; j < numTraj; j++){
       
      //check that the ID matches j
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "TrajectoryID:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      iTemp = atoi(sTemp);
      if(iTemp != j){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %d got %d\n", j, iTemp);
        exit(1);
      }
      SBPL_Trajectory_t traj;

      //read in this trajectory's probability
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "TrajectoryProbability:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      traj.prob = atof(sTemp);
      if(traj.prob < 0 || traj.prob > 1){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected TrajectoryProbability on the interval [0,1] but got %f\n", traj.prob);
        exit(1);
      }
      trajProbSum += traj.prob;

      //read the number of intermediate points are given for the trajectory
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "NumberOfPoints:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      int numPoints = atoi(sTemp);

      //for each point
      int prev_t = 0;
      for(int k=0; k < numPoints; k++){
        //fill in the point
        SBPL_Traj_Pt_t pt;
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.x = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.y = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        //pt.t = CONTTIME2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.timeResolution);
        pt.t = CONTTIME2DISC(atof(sTemp),timeRes);

        if(prev_t > pt.t && k != 0){
          printf("ERROR: dynamic obstacle file has incorrect format\n");
          printf("dynamic obstacle trajectory times can't decrease!\n");
          exit(1);
        }
        prev_t = pt.t;

        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.std_dev = atof(sTemp);

        //store the point in the trajectory
        traj.points.push_back(pt);
      }

      //check if the obstacle should "disappear" after it has finished its trajectory
      //or if it sits in the configuration from the last frame of the trajectory forever
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "ObstacleExistsAfterTrajectory:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      traj.existsAfter = atoi(sTemp);
      if(traj.existsAfter != 0 && traj.existsAfter != 1){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("ObstacleExistsAfterTrajectory is a boolean and needs to be 0 or 1\n");
        exit(1);
      }

      //store the trajectory in the dynamic obstacle
      obs.trajectories.push_back(traj);
    }

    //check that the trajectory probabilities sum to 1
    if(fabs(trajProbSum - 1.0) > ERR_EPS){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Probabilities for trajectories of dynamic obstacle %d sum to %f instead of 1\n", i, trajProbSum);
      exit(1);
    }

    //store the dynamic obstacle into the dynamic obstacle vector
    dynamicObstacles.push_back(obs);

  }
/*
  for(int i=0; i<dynamicObstacles.size(); i++){
    printf("obs %d: radiusSquared=%f\n",i,dynamicObstacles[i].radiusSquared);
    for(int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      printf("  traj %d: prob=%f\n",j,dynamicObstacles[i].trajectories[j].prob);
      for(int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        printf("    point %d: x=%f y=%f t=%d std_dev=%f\n",k,p.x,p.y,p.t,p.std_dev);
      }
    }
  }
  */
  

  //printf("Done Reading Dynamic Obstacles\n");
  fclose(fDynObs);
  return dynamicObstacles;
}


int planxythetatimelat(char* env_filename, char* mprim_filename, vector<SBPL_DynamicObstacle_t> dynObs, vector< vector<double> >& stats, int test_idx){

	int bRet = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.clear();
  /*
	sbpl_2Dpt_t pt_m;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
  */
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

  int eps_idx = stats.size()-1;
  for(double eps=initialEps; eps>0.999; eps -= dec_eps){
    printf("eps = %f\n", eps);

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentNAVXYTHETATIMELAT environment_navxythetatimelat;

    if(!environment_navxythetatimelat.InitializeEnv(env_filename, perimeterptsV, mprim_filename)){
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
    if(!environment_navxythetatimelat.setDynamicObstacles(dynObs, false)){
      printf("The start state is in collision!\nSolution does not exist\n\n");
      stats[eps_idx][4] = false;
      eps_idx--;
      continue;
    }

    //initialize planner
    vector<int> solution_stateIDs_V;
    WeightedAStar planner(&environment_navxythetatimelat, bforwardsearch);

    planner.set_initialsolution_eps(eps);

    //set search mode
    planner.set_search_mode(bsearchuntilfirstsolution);

    if(planner.set_start(environment_navxythetatimelat.getStartID()) == 0){
        printf("ERROR: failed to set start state\n");
        exit(1);
    }
    if(planner.set_goal(environment_navxythetatimelat.getGoalID()) == 0){
        printf("ERROR: failed to set goal state\n");
        exit(1);
    }


    printf("start planning...\n");
    try{
      bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    }
    catch(SBPL_Exception* e){
      printf("EXCEPTION: in XYTHETATIME!\n");
      for(;eps_idx >= 0; eps_idx--){
        stats[eps_idx][0] = -1;
        stats[eps_idx][1] = -1;
        stats[eps_idx][2] = -1;
        stats[eps_idx][3] = -1;
        stats[eps_idx][4] = 0;
      }
      return 0;
    }
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetatimelat.PrintTimeStat(stdout);

    vector<SBPL_4Dpt_t> xythetatimePath;
    environment_navxythetatimelat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
    printf("solution size=%d\n", (int)xythetatimePath.size());
    /*
    FILE* fSol = fopen("sol.txt", "w");
    for(unsigned int i = 0; i < xythetatimePath.size(); i++)
      fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
    fclose(fSol);
    */

    environment_navxythetatimelat.PrintTimeStat(stdout);
    int collisions = 0;
    for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
      SBPL_4Dpt_t pt = xythetatimePath[pind];
      collisions += getNumDynObsCollisions(dynObs,
          DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
          DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
          CONTTIME2DISC(pt.t,timeRes));
    }
    printf("Number of collisions with dynamic obstacles = %d\n", collisions);

    if(bRet)
      printf("Solution is found\n");
    else
      printf("Solution does not exist\n");

    printf("\n");
    fflush(NULL);

    bool solutionFound;
    int numExpands;
    int solutionCost;
    double searchTime;
    planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime);
    stats[eps_idx][0] = numExpands;
    stats[eps_idx][1] = solutionCost;
    stats[eps_idx][2] = searchTime;
    stats[eps_idx][3] = collisions;
    stats[eps_idx][4] = solutionFound;
    eps_idx--;
  }

  return bRet;
}

int planxythetatimebubblelat(char* env_filename, char* mprim_filename, vector<SBPL_DynamicObstacle_t> dynObs, vector< vector<double> >& stats, int test_idx){

	int bRet = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.clear();
  /*
	sbpl_2Dpt_t pt_m;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
  */
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

  int eps_idx = stats.size()-1;
  for(double eps=initialEps; eps>0.999; eps -= dec_eps){
    printf("eps = %f\n", eps);

    //Initialize Environment (should be called before initializing anything else)
    EnvDBubbleLat environment_navxythetatimebubblelat;

    if(!environment_navxythetatimebubblelat.InitializeEnv(env_filename, perimeterptsV, mprim_filename)){
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
    environment_navxythetatimebubblelat.setDynamicObstacles(dynObs);

    //initialize planner
    vector<int> solution_stateIDs_V;
    WeightedAStar planner(&environment_navxythetatimebubblelat, bforwardsearch);

    planner.set_initialsolution_eps(eps);

    //set search mode
    planner.set_search_mode(bsearchuntilfirstsolution);

    if(planner.set_start(environment_navxythetatimebubblelat.getStartID()) == 0){
        printf("ERROR: failed to set start state\n");
        exit(1);
    }
    if(planner.set_goal(environment_navxythetatimebubblelat.getGoalID()) == 0){
        printf("ERROR: failed to set goal state\n");
        exit(1);
    }

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navxythetatimebubblelat.PrintTimeStat(stdout);

    vector<SBPL_4Dpt_t> xythetatimePath;
    environment_navxythetatimebubblelat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
    /*
    FILE* fSol = fopen("sol.txt", "w");
    printf("solution size=%d\n", (int)xythetatimePath.size());
    for(unsigned int i = 0; i < xythetatimePath.size(); i++)
      fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
    fclose(fSol);
    */

    environment_navxythetatimebubblelat.PrintTimeStat(stdout);
    int collisions = 0;
    for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
      SBPL_4Dpt_t pt = xythetatimePath[pind];
      collisions += getNumDynObsCollisions(dynObs,
          DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
          DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
          CONTTIME2DISC(pt.t,timeRes));
    }
    printf("Number of collisions with dynamic obstacles = %d\n", collisions);

    if(bRet)
      printf("Solution is found\n");
    else
      printf("Solution does not exist\n");

    printf("\n");
    fflush(NULL);

    //environment_navxythetatimebubblelat.dumpStatesToFile();

    bool solutionFound;
    int numExpands;
    int solutionCost;
    double searchTime;
    planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime);
    stats[eps_idx][0] = numExpands;
    stats[eps_idx][1] = solutionCost;
    stats[eps_idx][2] = searchTime;
    stats[eps_idx][3] = collisions;
    stats[eps_idx][4] = solutionFound;
    eps_idx--;
  }

    return bRet;
}


int plansipplat(char* env_filename, char* mprim_filename, vector<SBPL_DynamicObstacle_t> dynObs, vector< vector<double> >& stats, int test_idx){

	int bRet = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.clear();
  /*
	sbpl_2Dpt_t pt_m;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
  */
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

  int eps_idx = stats.size()-1;
  for(double eps=initialEps; eps>0.999; eps -= dec_eps){
    printf("eps = %f\n", eps);

    //Initialize Environment (should be called before initializing anything else)
    EnvSIPPLat environment_intervallat;

    if(!environment_intervallat.InitializeEnv(env_filename, perimeterptsV, mprim_filename)){
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
    if(!environment_intervallat.setDynamicObstacles(dynObs, false)){
      printf("The start state is in collision!\nSolution does not exist\n\n");
      stats[eps_idx][4] = false;
      eps_idx--;
      continue;
    }

    //initialize planner
    vector<int> solution_stateIDs_V;
    WeightedAStar planner(&environment_intervallat, bforwardsearch);

    planner.set_initialsolution_eps(eps);

    //set search mode
    planner.set_search_mode(bsearchuntilfirstsolution);

    if(planner.set_start(environment_intervallat.getStartID()) == 0){
        printf("ERROR: failed to set start state\n");
        exit(1);
    }
    if(planner.set_goal(environment_intervallat.getGoalID()) == 0){
        printf("ERROR: failed to set goal state\n");
        exit(1);
    }

    printf("start planning...\n");
    bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_intervallat.PrintTimeStat(stdout);

    char buf[32];
    sprintf(buf, "temp/solutions/sol%.4d.txt", test_idx);
    FILE* fSol = fopen(buf, "w");
    vector<SBPL_4Dpt_t> xythetatimePath;
    environment_intervallat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
    printf("solution size=%d\n", (int)xythetatimePath.size());
    for(unsigned int i = 0; i < xythetatimePath.size(); i++){
      fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
    }
    fclose(fSol);

    environment_intervallat.PrintTimeStat(stdout);
    int collisions = 0;
    for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
      SBPL_4Dpt_t pt = xythetatimePath[pind];
      int temp = getNumDynObsCollisions(dynObs,
          DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
          DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
          CONTTIME2DISC(pt.t,timeRes));
      if(temp > 0){
        printf("id=%d\n",environment_intervallat.GetStateFromCoord(CONTXY2DISC(pt.x, 0.025), CONTXY2DISC(pt.x, 0.025), ContTheta2Disc(pt.theta, ENVINTERVALLAT_THETADIRS), CONTTIME2DISC(pt.t,timeRes)));
      }
      collisions += temp;
      //if(getNumDynObsCollisions(ReadDynamicObstacles(dynObs_filename), pt.x, pt.y, CONTTIME2DISC(pt.t,timeRes)))
      //printf("%d\n",pind);
    }
    printf("Number of collisions with dynamic obstacles = %d\n", collisions);

    //print a path
    if(bRet)
      printf("Solution is found\n");
    else
      printf("Solution does not exist\n");

    printf("\n");
    fflush(NULL);

    //environment_intervallat.dumpStatesToFile();

    bool solutionFound;
    int numExpands;
    int solutionCost;
    double searchTime;
    planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime);
    stats[eps_idx][0] = numExpands;
    stats[eps_idx][1] = solutionCost;
    stats[eps_idx][2] = searchTime;
    stats[eps_idx][3] = collisions;
    stats[eps_idx][4] = solutionFound;
    eps_idx--;
  }

    return bRet;
}

int planweightedinterval(char* env_filename, char* mprim_filename, vector<SBPL_DynamicObstacle_t> dynObs, vector< vector<double> >& stats, int test_idx){

	int bRet = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.clear();
  /*
	sbpl_2Dpt_t pt_m;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
  */
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

  int eps_idx = stats.size()-1;
  for(double eps=initialEps; eps>0.999; eps -= dec_eps){
    printf("eps = %f\n", eps);

    //Initialize Environment (should be called before initializing anything else)
    EnvIntervalLat environment_intervallat;

    if(!environment_intervallat.InitializeEnv(env_filename, perimeterptsV, mprim_filename)){
      printf("ERROR: InitializeEnv failed\n");
      exit(1);
    }
    if(!environment_intervallat.setDynamicObstacles(dynObs, false)){
      printf("The start state is in collision!\nSolution does not exist\n\n");
      stats[eps_idx][4] = false;
      eps_idx--;
      continue;
    }

    //initialize planner
    vector<int> solution_stateIDs_V;
    IntervalPlanner planner(&environment_intervallat);

    planner.set_initialsolution_eps(eps);

    //set search mode
    planner.set_search_mode(bsearchuntilfirstsolution);

    if(planner.set_start(environment_intervallat.getStartID()) == 0){
        printf("ERROR: failed to set start state\n");
        exit(1);
    }
    if(planner.set_goal(environment_intervallat.getGoalID()) == 0){
        printf("ERROR: failed to set goal state\n");
        exit(1);
    }

    printf("start planning...\n");
    try{
      bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    }
    catch(SBPL_Exception* e){
      printf("EXCEPTION: in INTERVAL PLANNER!\n");
      for(;eps_idx >= 0; eps_idx--){
        stats[eps_idx][0] = -1;
        stats[eps_idx][1] = -1;
        stats[eps_idx][2] = -1;
        stats[eps_idx][3] = -1;
        stats[eps_idx][4] = 0;
      }
      return 0;
    }
    printf("done planning\n");
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_intervallat.PrintTimeStat(stdout);

    char buf[32];
    sprintf(buf, "temp/solutions/sol%.4d.txt", test_idx);
    FILE* fSol = fopen(buf, "w");
    vector<SBPL_4Dpt_t> xythetatimePath;
    environment_intervallat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
    printf("solution size=%d\n", (int)xythetatimePath.size());
    for(unsigned int i = 0; i < xythetatimePath.size(); i++){
      fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
    }
    fclose(fSol);

    environment_intervallat.PrintTimeStat(stdout);
    int collisions = 0;
    for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
      SBPL_4Dpt_t pt = xythetatimePath[pind];
      int temp = getNumDynObsCollisions(dynObs,
          DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
          DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
          CONTTIME2DISC(pt.t,timeRes));
      if(temp > 0){
        printf("id=%d\n",environment_intervallat.GetStateFromCoord(CONTXY2DISC(pt.x, 0.025), CONTXY2DISC(pt.x, 0.025), ContTheta2Disc(pt.theta, ENVINTERVALLAT_THETADIRS), CONTTIME2DISC(pt.t,timeRes)));
      }
      collisions += temp;
      //if(getNumDynObsCollisions(ReadDynamicObstacles(dynObs_filename), pt.x, pt.y, CONTTIME2DISC(pt.t,timeRes)))
      //printf("%d\n",pind);
    }
    printf("Number of collisions with dynamic obstacles = %d\n", collisions);

    //print a path
    if(bRet)
      printf("Solution is found\n");
    else
      printf("Solution does not exist\n");

    printf("\n");
    fflush(NULL);

    //environment_intervallat.dumpStatesToFile();

    bool solutionFound;
    vector<int> numExpands;
    vector<int> solutionCost;
    vector<double> searchTime;
    vector<double> searchEps;
    planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime, &searchEps);
    if(numExpands.empty()){
      stats[eps_idx][0] = -1;
      stats[eps_idx][1] = -1;
      stats[eps_idx][2] = -1;
      stats[eps_idx][3] = collisions;
      stats[eps_idx][4] = solutionFound;
    }
    else{
      stats[eps_idx][0] = numExpands[0];
      stats[eps_idx][1] = solutionCost[0];
      stats[eps_idx][2] = searchTime[0];
      stats[eps_idx][3] = collisions;
      stats[eps_idx][4] = solutionFound;
    }
    eps_idx--;
  }

    return bRet;
}

int planintervallat(char* env_filename, char* mprim_filename, vector<SBPL_DynamicObstacle_t> dynObs, vector< vector<double> >& stats, int test_idx){

	int bRet = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_2Dpt_t> perimeterptsV;
	perimeterptsV.clear();
  /*
	sbpl_2Dpt_t pt_m;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	perimeterptsV.push_back(pt_m);
  */
	

	//clear perimeter
	//perimeterptsV.clear();
	//pt_m.x = 0.0;
	//pt_m.y = 0.0;
	//perimeterptsV.push_back(pt_m);

  //Initialize Environment (should be called before initializing anything else)
  EnvIntervalLat environment_intervallat;

  if(!environment_intervallat.InitializeEnv(env_filename, perimeterptsV, mprim_filename)){
    printf("ERROR: InitializeEnv failed\n");
    exit(1);
  }
  if(!environment_intervallat.setDynamicObstacles(dynObs, false)){
    printf("The start state is in collision!\nSolution does not exist\n\n");
    for(unsigned int i=0; i<stats.size(); i++)
      stats[i][4] = false;
    //continue;
    return 0;
  }

  //initialize planner
  vector<int> solution_stateIDs_V;
  IntervalPlanner planner(&environment_intervallat);

  planner.set_initialsolution_eps(initialEps);

  //set search mode
  planner.set_search_mode(bsearchuntilfirstsolution);

  if(planner.set_start(environment_intervallat.getStartID()) == 0){
      printf("ERROR: failed to set start state\n");
      exit(1);
  }
  if(planner.set_goal(environment_intervallat.getGoalID()) == 0){
      printf("ERROR: failed to set goal state\n");
      exit(1);
  }

  printf("start planning...\n");
  bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
  printf("done planning\n");
  std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

  environment_intervallat.PrintTimeStat(stdout);

  char buf[32];
  sprintf(buf, "temp/solutions/sol%.4d.txt", test_idx);
  FILE* fSol = fopen(buf, "w");
  vector<SBPL_4Dpt_t> xythetatimePath;
  environment_intervallat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
  printf("solution size=%d\n", (int)xythetatimePath.size());
  for(unsigned int i = 0; i < xythetatimePath.size(); i++){
    fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
  }
  fclose(fSol);

  environment_intervallat.PrintTimeStat(stdout);
  int collisions = 0;
  for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
    SBPL_4Dpt_t pt = xythetatimePath[pind];
    int temp = getNumDynObsCollisions(dynObs,
        DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
        DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
        CONTTIME2DISC(pt.t,timeRes));
    if(temp > 0){
      printf("id=%d\n",environment_intervallat.GetStateFromCoord(CONTXY2DISC(pt.x, 0.025), CONTXY2DISC(pt.x, 0.025), ContTheta2Disc(pt.theta, ENVINTERVALLAT_THETADIRS), CONTTIME2DISC(pt.t,timeRes)));
    }
    collisions += temp;
    //if(getNumDynObsCollisions(ReadDynamicObstacles(dynObs_filename), pt.x, pt.y, CONTTIME2DISC(pt.t,timeRes)))
    //printf("%d\n",pind);
  }
  printf("Number of collisions with dynamic obstacles = %d\n", collisions);

  //print a path
  if(bRet)
    printf("Solution is found\n");
  else
    printf("Solution does not exist\n");

  printf("\n");
  fflush(NULL);

  //environment_intervallat.dumpStatesToFile();

  bool solutionFound;
  vector<int> numExpands;
  vector<int> solutionCost;
  vector<double> searchTime;
  vector<double> searchEps;
  planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime, &searchEps);
  int s=stats.size()-1;
  int i;
  for(i=0; i<(int)searchEps.size(); i++){
    stats[s-i][0] = numExpands[i];
    stats[s-i][1] = solutionCost[i];
    stats[s-i][2] = searchTime[i];
    stats[s-i][3] = collisions;
    stats[s-i][4] = solutionFound;
  }
  for(; s-i>=0; i++)
    stats[s-i][4] = false;

  return bRet;
}

void computeStats(vector< vector< vector<int> > > expands, //first vector is for planners, the second is for the epsilons, and the third is for the test
                  vector< vector< vector<int> > > g, //first vector is for planners, the second is for the epsilons, and the third is for the test
                  vector< vector< vector<double> > > time, //first vector is for planners, the second is for the epsilons, and the third is for the test
                  vector< vector< vector<int> > > collisions, //first vector is for planners, the second is for the epsilons, and the third is for the test
                  vector< vector< vector<int> > > solution_found, //first vector is for planners, the second is for the epsilons, and the third is for the test
                  vector<char*> name){ //first vector is for planners
  //compute stats on data
  printf("\n\n\n\nComputing statistics...\n");

  if(solution_found.empty() || solution_found[0].empty() || solution_found[0][0].empty()){
    printf("No tests were run!\n");
    exit(1);
  }

  int numPlanners = solution_found.size();
  int numEps = solution_found[0].size();
  int numTests = solution_found[0][0].size();

  vector< vector<bool> > allFound;
  allFound.resize(numEps);
  for(int j=0; j<numEps; j++){
    allFound[j].resize(numTests);
    for(int k=0; k<numTests; k++){
      bool solFound = true;
      for(int i=0; i<numPlanners; i++){
        solFound &= solution_found[i][j][k];
      }
      allFound[j][k] = solFound;
    }
  }

  printf("%d tests were run on the following planners\n", numTests);
  for(int m=0; m<2; m++){
    if(m==0)
      printf("\n\nStats computed for each planner individually\n");
    else
      printf("\n\nStats computed only for paths that all planners found\n");

    for(int i=0; i<numPlanners; i++){
      printf("\n%s\n",name[i]);

      for(int j=numEps-1; j>=0; j--){
        double mean_expands = 0;
        double mean_g = 0;
        double mean_time = 0;
        double mean_collisions = 0;
        int total_sol_found = 0;
        for(int k=0; k<numTests; k++){
          if((m==0 && solution_found[i][j][k] == 1) || 
             (m==1 && allFound[j][k])){
            mean_expands += expands[i][j][k];
            mean_g += g[i][j][k];
            mean_time += time[i][j][k];
            mean_collisions += (collisions[i][j][k] > 0 ? 1 : 0);
            total_sol_found += solution_found[i][j][k];
          }
        }
        mean_expands /= total_sol_found;
        mean_g /= total_sol_found;
        mean_time /= total_sol_found;
        mean_collisions /= total_sol_found;
        printf("eps=%f avg_expands=%f avg_g=%f avg_time=%f avg_collisions=%f solutionsFound=%d\n", (j*dec_eps)+1.0, 
            mean_expands, mean_g, mean_time, mean_collisions, total_sol_found);
        
#if PRINT_SPREAD_STATS
        double std_dev_expands = 0;
        double std_dev_g = 0;
        double std_dev_time = 0;
        double std_dev_collisions = 0;
        for(int k=0; k<numTests; k++){
          if((m==0 && solution_found[i][j][k] == 1) || 
             (m==1 && allFound[j][k])){
            std_dev_expands += (expands[i][j][k]-mean_expands)*(expands[i][j][k]-mean_expands);
            std_dev_g += (g[i][j][k]-mean_g)*(g[i][j][k]-mean_g);
            std_dev_time += (time[i][j][k]-mean_time)*(time[i][j][k]-mean_time);
            std_dev_collisions += (collisions[i][j][k]-mean_collisions)*(collisions[i][j][k]-mean_collisions);
          }
        }
        std_dev_expands = sqrt(std_dev_expands/total_sol_found);
        std_dev_g = sqrt(std_dev_g/total_sol_found);
        std_dev_time = sqrt(std_dev_time/total_sol_found);
        std_dev_collisions = sqrt(std_dev_collisions/total_sol_found);

        printf("       std_dev_expands=%f std_dev_g=%f std_dev_time=%f std_dev_collisions=%f\n", std_dev_expands, std_dev_g, std_dev_time, std_dev_collisions); 
#endif
      }

    }
  }
}

int main(int argc, char *argv[]){
  char* mprimFile = argv[1];
  vector<char*> name;

  //Pick which planners to run
  name.push_back("XYThetaTimeLattice");
  //name.push_back("XYThetaTimeBubbleLattice");
  //name.push_back("SIPPLattice");
  name.push_back("IntervalLattice");

  vector< vector< vector<int> > > expands;
  vector< vector< vector<int> > > g;
  vector< vector< vector<double> > > time;
  vector< vector< vector<int> > > collisions;
  vector< vector< vector<int> > > solution_found;

  int numPlanners = name.size();
  int numEps = (int)((initialEps-1.0)/dec_eps+1 + 0.5);
  int numTests = (argc-2)/2;

  expands.resize(numPlanners);
  g.resize(numPlanners);
  time.resize(numPlanners);
  collisions.resize(numPlanners);
  solution_found.resize(numPlanners);
  for(int i=0; i<numPlanners; i++){
    expands[i].resize(numEps);
    g[i].resize(numEps);
    time[i].resize(numEps);
    collisions[i].resize(numEps);
    solution_found[i].resize(numEps);
    for(int j=0; j<numEps; j++){
      expands[i][j].resize(numTests);
      g[i][j].resize(numTests);
      time[i][j].resize(numTests);
      collisions[i][j].resize(numTests);
      solution_found[i][j].resize(numTests);
      for(int k=0; k<numTests; k++){
        expands[i][j][k] = -1;
        g[i][j][k] = -1;
        time[i][j][k] = -1;
        collisions[i][j][k] = -1;
        solution_found[i][j][k] = -1;
      }
    }
  }

  vector< vector<double> > stats;
  stats.resize(numEps);
  for(int i=0; i<numEps; i++)
    stats[i].resize(5);

  for(int i=0;i<argc;i++)
    printf("%s\n",argv[i]);

  for(int i=0; i<numTests; i++){
    char* envFile = argv[i+2];
    char* dynObsFile = argv[i+2+numTests];
    unsigned int p=0;

    printf("files %s %s \n",envFile,dynObsFile);

    vector<SBPL_DynamicObstacle_t> dynObs = ReadDynamicObstacles(dynObsFile);


    if(p<name.size() && strncmp("XYThetaTimeLattice",name[p],18)==0){
      //xythetatime planning
      printf("\n\n\nTest %d of XYThetaTimeLattice Planner\n", i);
      planxythetatimelat(envFile, mprimFile, dynObs, stats, i);
      for(unsigned int j=0; j<stats.size(); j++){
        expands[p][j][i] = (int)(stats[j][0]+0.5);
        g[p][j][i] = (int)(stats[j][1]+0.5);
        time[p][j][i] = stats[j][2];
        collisions[p][j][i] = (int)(stats[j][3]+0.5);
        solution_found[p][j][i] = (int)(stats[j][4]+0.5);
      }
      p++;
    }

    if(p<name.size() && strncmp("XYThetaTimeBubbleLattice",name[p],24)==0){
      //xythetatimebubble planning
      printf("\n\n\nTest %d of XYThetaTimeBubbleLattice Planner\n", i);
      planxythetatimebubblelat(envFile, mprimFile, dynObs, stats, i);
      for(unsigned int j=0; j<stats.size(); j++){
        expands[p][j][i] = (int)(stats[j][0]+0.5);
        g[p][j][i] = (int)(stats[j][1]+0.5);
        time[p][j][i] = stats[j][2];
        collisions[p][j][i] = (int)(stats[j][3]+0.5);
        solution_found[p][j][i] = (int)(stats[j][4]+0.5);
      }
      p++;
    }

    if(p<name.size() && strncmp("SIPPLattice",name[p],11)==0){
      //interval planning
      printf("\n\n\nTest %d of SIPPLattice Planner\n", i);
      plansipplat(envFile, mprimFile, dynObs, stats, i);
      for(unsigned int j=0; j<stats.size(); j++){
        for(unsigned int k=0; k<stats[j].size(); k++){
          expands[p][j][i] = (int)(stats[j][0]+0.5);
          g[p][j][i] = (int)(stats[j][1]+0.5);
          time[p][j][i] = stats[j][2];
          collisions[p][j][i] = (int)(stats[j][3]+0.5);
          solution_found[p][j][i] = (int)(stats[j][4]+0.5);
        }
      }
      p++;
    }

    if(p<name.size() && strncmp("IntervalLattice",name[p],15)==0){
      //interval planning
      printf("\n\n\nTest %d of IntervalLattice Planner\n", i);
      //planintervallat(envFile, mprimFile, dynObs, stats, i);
      planweightedinterval(envFile, mprimFile, dynObs, stats, i);
      for(unsigned int j=0; j<stats.size(); j++){
        expands[p][j][i] = (int)(stats[j][0]+0.5);
        g[p][j][i] = (int)(stats[j][1]+0.5);
        time[p][j][i] = stats[j][2];
        collisions[p][j][i] = (int)(stats[j][3]+0.5);
        solution_found[p][j][i] = (int)(stats[j][4]+0.5);
      }
      p++;
    }
  }

  char buf[64];
  for(unsigned int i=0; i<name.size(); i++){
    sprintf(buf,"temp/results/data_%s.csv",name[i]);
    FILE* fout = fopen(buf,"w");
    for(int j=0; j<numTests; j++){
      for(int k=0; k<numEps; k++){
        if(solution_found[i][k][j])
          fprintf(fout,"%d,%f,%d",expands[i][k][j],time[i][k][j],g[i][k][j]);
        else
          fprintf(fout,"-1,-1,-1");
        if(k+1<numEps)
          fprintf(fout,",");
      }
      fprintf(fout,"\n");
    }
    fclose(fout);
  }

  //computeStats(expands, g, time, collisions, solution_found, name);
	
	return 0;
}

