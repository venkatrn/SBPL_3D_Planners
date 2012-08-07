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
#include <time.h>
#include <sbpl_dynamic_planner_3D/envInterval.h>
#include <sbpl_dynamic_planner_3D/envDBubble.h>
#include <sbpl_dynamic_planner_3D/envTime.h>
#include <sbpl_dynamic_planner_3D/weightedAStar.h>
#include <sbpl_dynamic_planner_3D/intervalPlanner.h>

FILE* file;

double time_resolution = 0.1;

void PrintUsage(char *argv[])
{
	printf("USAGE: %s <cfg file>\n", argv[0]);
}

/* This version of Anytime SIPP does not support incremental search - i.e, extending the time horizon*. collisionCheck is deprecated.*/
bool collisionCheck(vector<SBPL_4Dpt_t> xythetatimePath, vector<vector<vector <double> > > dynObsTrajs, vector<double> dynObsRad, double robot_radius, vector<bool>& marked_obstacles, vector<SBPL_DynamicObstacle_t>& dynObs){


  //TODO: optimize code (use time pointers to run through the trajectories efficiently)
  bool atleast_one_collision = false;
  int obsCount = 0;

  for(int i=0;i<(int)xythetatimePath.size();i++){
	for(int j=0;j<(int)dynObsTrajs.size();j++){
		for(int k=0;k<(int)(dynObsTrajs[j].size()-1);k++){		
			if((dynObsTrajs[j][k][0]-xythetatimePath[i].x)*(dynObsTrajs[j][k][0]-xythetatimePath[i].x)+(dynObsTrajs[j][k][1]-xythetatimePath[i].y)*(dynObsTrajs[j][k][1]-xythetatimePath[i].y)+(dynObsTrajs[j][k][2]-xythetatimePath[i].z)*(dynObsTrajs[j][k][2]-xythetatimePath[i].z) <= (robot_radius+dynObsRad[j])*(robot_radius+dynObsRad[j]) && marked_obstacles[j]==false){	
				  if(abs(dynObsTrajs[j][k][3]-xythetatimePath[i].t)<=0.1){
					// Add obstacle trajectory for the next SIPP iteration
					marked_obstacles[j] = true;
					obsCount++;
					SBPL_DynamicObstacle_t tempObs;
					tempObs.trajectories.resize(1);
					tempObs.trajectories[0].prob = 1;
					tempObs.trajectories[0].points.resize((int)dynObsTrajs[j].size());
					for(int l=0;l<(int)(dynObsTrajs[j].size());l++){
						tempObs.trajectories[0].points[l].x = dynObsTrajs[j][l][0];
						tempObs.trajectories[0].points[l].y = dynObsTrajs[j][l][1];
						tempObs.trajectories[0].points[l].z = dynObsTrajs[j][l][2];
						tempObs.trajectories[0].points[l].t = (int)(dynObsTrajs[j][l][3]/time_resolution+0.5);
						tempObs.trajectories[0].points[l].std_dev = dynObsTrajs[j][l][4];
						//ROS_INFO("Points: %d %d %d %lf %lf %lf %lf\n",i,j,k,tempObs.trajectories[0].points[l].x,tempObs.trajectories[0].points[l].y,tempObs.trajectories[0].points[l].z,tempObs.trajectories[0].points[l].t);
					}
					tempObs.trajectories[0].existsAfter = 0;
					tempObs.radius = dynObsRad[j];
					//dynObs.push_back(tempObs);
					dynObs[j] = tempObs;
					atleast_one_collision = true;
					break;
				}
			}
		}
	}
  }
  /*
  if(atleast_one_collision){
	ROS_INFO("Collision !!\n");
	ROS_INFO("We will now consider %d more dynamic obstacles for the next iteration\n",obsCount);
  }
  else{
	ROS_INFO("No collision. We are done :)\n");
	}
*/
  return atleast_one_collision;
}


bool initializeDynamicObstacles(double time_horizon, vector<vector<vector <double> > > dynObsTrajs, vector<double> dynObsRad, vector<bool>& marked_obstacles, vector<SBPL_DynamicObstacle_t>& dynObs){

  for(int j=0;j<(int)dynObsTrajs.size();j++){
		//if(marked_obstacles[j]==true)
		//	continue;
		for(int k=0;k<(int)(dynObsTrajs[j].size());k++){
			if(dynObsTrajs[j][k][3] <= time_horizon && k < (int)(dynObsTrajs[j].size()-1))
				continue;
			//if(k>0){
			{
				SBPL_DynamicObstacle_t tempObs;
				tempObs.trajectories.resize(1);
				tempObs.trajectories[0].prob = 1;
				tempObs.trajectories[0].points.resize((int)dynObsTrajs[j].size());
				for(int l=0;l<k;l++){
					tempObs.trajectories[0].points[l].x = dynObsTrajs[j][l][0];
					tempObs.trajectories[0].points[l].y = dynObsTrajs[j][l][1];
					tempObs.trajectories[0].points[l].z = dynObsTrajs[j][l][2];
					tempObs.trajectories[0].points[l].t = (int)(dynObsTrajs[j][l][3]/time_resolution+0.5);
					tempObs.trajectories[0].points[l].std_dev = dynObsTrajs[j][l][4];
				}
				tempObs.trajectories[0].existsAfter = 0;
				tempObs.radius = dynObsRad[j];
				dynObs.push_back(tempObs);
				break;		
			}
			
  		}
  }
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

bool getPoint(FILE* ptr, vector<double>& point){
  if(fscanf(ptr,"%lf %lf %lf %lf %lf",&point[0], &point[1], &point[2], &point[3], &point[4])==EOF)
	return false;
  return true;
}


bool readDynObsTrajs(vector<vector<vector <double> > >& dynObsTrajs, vector<double>& dynObsRad, char* file_path){
  FILE* ptr;
  ptr = fopen(file_path,"r");
  if(ptr == NULL){
	SBPL_ERROR("Cannot open dynObs file");
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
	SBPL_ERROR("skip1 Error reading DynObs file");
	return 0;
  	}
  }
  fclose(ptr);
  
  return true;
}

int planAnytimeIntervallat(int argc, char *argv[], char* env_path, char* dyn_obs_path, double eps, bool bsearchuntilfirstsolution, bool uptotimehorizon, double time_horizon_secs, double total_allocated_time_secs, double SIPP_allocated_time_secs)
{	


	int bRet = 0;
	bool in_collision = true;
	vector<vector<vector <double> > > dynObsTrajs;
	vector<double> dynObsRad;
	vector<SBPL_DynamicObstacle_t> dynObs;

	clock_t start_time,current_time;
	vector<bool> marked_obstacles;

	double total_search_time;
	clock_t search_begin_time, search_end_time;
	search_begin_time = clock();
	double totalTime = 0;
	int totalExpands = 0, bestSolutionCost = 0, solutionSize = 0;

	//set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	vector<sbpl_3Dpt_t> perimeterptsV;
	sbpl_3Dpt_t pt_m;
	double halfwidth = 0.2; //0.335; 
	double halflength = 0.2; //0.335; 
	double halfdepth = 0.2; //0.335;
	pt_m.x = -halflength;
	pt_m.y = -halfwidth;
	pt_m.z = halfdepth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = -halfwidth;
	pt_m.z = halfdepth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = halflength;
	pt_m.y = halfwidth;
	pt_m.z = halfdepth;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -halflength;
	pt_m.y = halfwidth;
	pt_m.z = halfdepth;
	perimeterptsV.push_back(pt_m);
	
	// Read dynObs trajectories
	readDynObsTrajs(dynObsTrajs, dynObsRad, dyn_obs_path);
	marked_obstacles.resize((int)dynObsTrajs.size());

	
	// Initialize all dynamic obstacles with trajectories limited to time horizon
	initializeDynamicObstacles(time_horizon_secs, dynObsTrajs, dynObsRad, marked_obstacles, dynObs);

        start_time = clock();
	

	while(in_collision == true && (clock()-start_time)<CLOCKS_PER_SEC*(total_allocated_time_secs-SIPP_allocated_time_secs)){
		
		
				    
	        EnvIntervalLat environment_navIntervallat_dynamic;
	
		if(!environment_navIntervallat_dynamic.InitializeEnv(env_path, perimeterptsV, argv[2])){
		printf("ERROR: InitializeEnv failed\n");
		return 0;
		//exit(1);
		}
	
		
		// Set dynamic obstacles
		if(!environment_navIntervallat_dynamic.setDynamicObstacles(dynObs, true))
			printf("ERROR: Failed to set dynamic obstacles\n");


		double robot_radius = environment_navIntervallat_dynamic.GetRobotRadius();
		//initialize planner

		IntervalPlanner planner(&environment_navIntervallat_dynamic);

		planner.set_initialsolution_eps(eps);

		//set search mode
		planner.set_search_mode(bsearchuntilfirstsolution);
		vector<int> solution_stateIDs_V;

		  if(planner.set_start(environment_navIntervallat_dynamic.getStartID()) == 0)
		  {
		      printf("ERROR: failed to set start state\n");
		      exit(1);
		  }

		  if(planner.set_goal(environment_navIntervallat_dynamic.getGoalID()) == 0)
		  {
		      printf("ERROR: failed to set start state\n");
		      exit(1);
		  }



 		    planner.force_planning_from_scratch();
		    //solution_stateIDs_V.clear();
		    printf("start planning...\n");
			bRet = planner.replan(SIPP_allocated_time_secs, &solution_stateIDs_V);
		    printf("done planning\n");
			std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

		    environment_navIntervallat_dynamic.PrintTimeStat(stdout);
/*
		    FILE* fSol = fopen("sol.txt", "w");
*/			vector<SBPL_4Dpt_t> xythetatimePath;
			environment_navIntervallat_dynamic.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
		/*	printf("solution size=%d\n", (int)xythetatimePath.size());
			for(unsigned int i = 0; i < xythetatimePath.size(); i++) {
				fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).z, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
			}
		    fclose(fSol);
		    environment_navIntervallat_dynamic.PrintTimeStat(stdout);
*/

			//print a path
			if(bRet)
			{
				//print the solution
				printf("Solution is found\n");
				// Planner statistics
				bool solutionFound;
				vector<int> numExpands;
				vector<int> solutionCost;
				vector<double> searchTime;
				vector<double> searchEps;
				solutionFound = 0;
				numExpands.clear();
				solutionCost.clear();
				searchTime.clear();
				searchEps.clear();
				planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime, &searchEps);
/*				printf("Solution Found = %d\n", solutionFound);   
				printf("Number of Expands: ");
				for(unsigned int i=0; i<numExpands.size(); i++)
					printf("%d ",numExpands[i]);
				printf("\nSolution Costs: ");
				for(unsigned int i=0; i<solutionCost.size(); i++)
					printf("%d ",solutionCost[i]);
				printf("\nSearch Time: ");
				for(unsigned int i=0; i<searchTime.size(); i++)
					printf("%f ",searchTime[i]);
			 	printf("\nSearch Epsilon: ");
				for(unsigned int i=0; i<searchEps.size(); i++)
					printf("%f ",searchEps[i]);
				printf("\n");
*/
/* 
				//Exp 1
				//Write to file - [searchEpsilon timeHorizon searchTime numExpands solutionSize solutionCost]				
				fprintf(file,"%lf %lf %f %d %d %d\n",searchEps[0], time_horizon_secs, searchTime[0], numExpands[0], (int)solution_stateIDs_V.size(), solutionCost[0]); 
*/

/*				//Exp 2
				//Write to file - [timeHorizon searchEpsilon searchTime numExpands solutionSize solutionCost]	
				for(unsigned int i=0; i<numExpands.size(); i++)		
					fprintf(file,"%lf %lf %f %d %d %d\n",time_horizon_secs, searchEps[i], searchTime[i], numExpands[i], (int)solution_stateIDs_V.size(), solutionCost[i]); 
*/
			        //Exp 3
				for(unsigned int i=0; i<numExpands.size(); i++){
					totalExpands = totalExpands + numExpands[i];
					totalTime = totalTime + searchTime[i];
					solutionSize = (int)solution_stateIDs_V.size();
				}
				bestSolutionCost = solutionCost[(int)solutionCost.size()-1];
				

				
			}
			else
				{printf("Solution does not exist\n");
				 return 0;
				  
				}

			fflush(NULL);

		    	environment_navIntervallat_dynamic.dumpStatesToFile();

			if(uptotimehorizon == true){
				in_collision = false;
				break;
			}
			
			in_collision = collisionCheck(xythetatimePath, dynObsTrajs, dynObsRad, robot_radius, marked_obstacles, dynObs);
			
		  	solution_stateIDs_V.clear();
		  	xythetatimePath.clear();

	}
	
    if(in_collision){
	search_end_time = clock();
	ROS_INFO("The planner could not find a safe path within the allocated time");
	return 0;
    }
    else{
	search_end_time = clock();
	total_search_time = double(search_end_time-search_begin_time)/CLOCKS_PER_SEC;
    	//Exp 3		
	//Write to file - [SIPP_Time initEpsilon timeHorizon totalSearchTime totalExpands solutionSize bestSolutionCost]		
	fprintf(file,"%lf %lf %lf %lf %d %d %d\n",SIPP_allocated_time_secs, eps, time_horizon_secs, total_search_time, totalExpands, solutionSize, bestSolutionCost); 
    }
    return bRet;
}

int main(int argc, char *argv[])
{
	
	if(argc < 2){
		PrintUsage(argv);
		exit(1);
	}

		double init_eps = 5;
		double eps_vec[2] = {5,10};
		double SIPP_time_vec[1] = {0.1};
		double total_allocated_time_secs = 3000.0; 
		double SIPP_allocated_time_secs = 0; //in seconds (1200)
		bool bsearchuntilfirstsolution = false;  // Plan till eps = 1
		bool uptotimehorizon = false;

	int numEnvironments = 50;
	for(int j=0;j<numEnvironments;j++){
		char env_path[100],dyn_obs_path[100],stats_path[100];
		sprintf(env_path,"test_envs/test%.4d.exp",j);
		sprintf(dyn_obs_path,"test_envs/dynObs%.4d.dob",j);
		sprintf(stats_path,"test_envs/stats%.4d.txt",j);


/*		

		// Exp 1

	    	double init_eps = 3.0;
		double eps_vec[4] = {1,3,5,10};
		double time_horizon_secs = 10;
		double total_allocated_time_secs = 3000.0; 
		double SIPP_allocated_time_secs = 1000.0; //in seconds (1200)
		bool bsearchuntilfirstsolution = true;
		bool uptotimehorizon = true;

		// Time for first solution upto time horizon vs Time Horizon (Parameters: epsilon) [SIPP time - 1000, Total Time - 3000]
		file = fopen(stats_path,"w");
		if(file == NULL){
			SBPL_ERROR("Unable to open file for writing");
			exit(1);
		}
		for(int i=0;i<4;i++){
			init_eps = eps_vec[i];
			for(double time_horizon_secs = 0;time_horizon_secs < 35;time_horizon_secs+=1){
				if(!planAnytimeIntervallat(argc, argv, env_path, dyn_obs_path, init_eps, bsearchuntilfirstsolution, uptotimehorizon, time_horizon_secs, total_allocated_time_secs, SIPP_allocated_time_secs))
					break;
			}
		}
		
		
*/

/*
		// Exp 2

		double init_eps = 5.0;
		double th_vec[4] = {5,15,25,35};
		double total_allocated_time_secs = 3000.0; 
		double SIPP_allocated_time_secs = 1000.0; //in seconds (1200)
		bool bsearchuntilfirstsolution = false;  // Plan till eps = 1
		bool uptotimehorizon = true;
		double time_horizon_secs = 0;

		// Time for first solution upto time horizon vs Time Horizon (Parameters: epsilon) [SIPP time - 1000, Total Time - 3000]
		file = fopen(stats_path,"w");
		if(file == NULL){
			SBPL_ERROR("Unable to open file for writing");
			exit(1);
		}
		for(int i=0;i<4;i++){
			time_horizon_secs = th_vec[i];
				if(!planAnytimeIntervallat(argc, argv, env_path, dyn_obs_path, init_eps, bsearchuntilfirstsolution, uptotimehorizon, time_horizon_secs, total_allocated_time_secs, SIPP_allocated_time_secs))
					break;
			
		}
*/
		// Exp 3


		// Time for first solution upto time horizon vs Time Horizon (Parameters: epsilon) [SIPP time - 1000, Total Time - 3000]
		file = fopen(stats_path,"w");
		if(file == NULL){
			SBPL_ERROR("Unable to open file for writing");
			exit(1);
		}
		
		for(int j=0;j<1;j++){
			SIPP_allocated_time_secs = SIPP_time_vec[j];
			for(int i=0;i<2;i++){
				init_eps = eps_vec[i];
				for(double time_horizon_secs = 0;time_horizon_secs < 35;time_horizon_secs+=1){
					if(!planAnytimeIntervallat(argc, argv, env_path, dyn_obs_path, init_eps, bsearchuntilfirstsolution, uptotimehorizon, time_horizon_secs, total_allocated_time_secs, SIPP_allocated_time_secs))
						break;
				}
			
			}
			}

		fclose(file);


	}

	return 0;
}





