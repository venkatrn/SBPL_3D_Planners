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
#include <sbpl_dynamic_planner/envInterval.h>
#include <sbpl_dynamic_planner/envDBubble.h>
#include <sbpl_dynamic_planner/envTime.h>
#include <sbpl_dynamic_planner/weightedAStar.h>
#include <sbpl_dynamic_planner/intervalPlanner.h>


double time_resolution = 0.1;

void PrintUsage(char *argv[])
{
	printf("USAGE: %s <cfg file>\n", argv[0]);
}

int planIntervallat(int argc, char *argv[], double eps)
{

	int bRet = 0;
	double allocated_time_secs = 20.0; //in seconds (1200)
	bool bsearchuntilfirstsolution = true;

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
	

	//Initialize Environment (should be called before initializing anything else)
	EnvIntervalLat environment_navIntervallat;

	/*if(argc >= 4){
		if(!environment_navxythetatimebubblelat.InitializeEnv(argv[2], perimeterptsV, argv[4], argv[5])){
			printf("ERROR: InitializeEnv failed\n");
			exit(1);
		}
	}*/
	if(argc >= 4){
		if(!environment_navIntervallat.InitializeEnv(argv[1], perimeterptsV, argv[2], argv[3])){
			printf("ERROR: InitializeEnv failed\n");
			exit(1);
		}
	}
	else{
    	printf("Please provide three arguments: environmentFile motionPrimitiveFile dynamicObstacleFile\n");
    	exit(1);
	}

	//initialize planner
	vector<int> solution_stateIDs_V;
	bool bforwardsearch = true;
	
	IntervalPlanner planner(&environment_navIntervallat);

	planner.set_initialsolution_eps(eps);

	//set search mode
	planner.set_search_mode(bsearchuntilfirstsolution);

  if(planner.set_start(environment_navIntervallat.getStartID()) == 0)
  {
      printf("ERROR: failed to set start state\n");
      exit(1);
  }


  if(planner.set_goal(environment_navIntervallat.getGoalID()) == 0)
  {
      printf("ERROR: failed to set start state\n");
      exit(1);
  }


    printf("start planning...\n");
	bRet = planner.replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
	std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_navIntervallat.PrintTimeStat(stdout);

    FILE* fSol = fopen("sol.txt", "w");
	vector<SBPL_4Dpt_t> xythetatimePath;
	environment_navIntervallat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
	printf("solution size=%d\n", (int)xythetatimePath.size());
	for(unsigned int i = 0; i < xythetatimePath.size(); i++) {
		fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).z, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
	}
    fclose(fSol);

    environment_navIntervallat.PrintTimeStat(stdout);

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
		planner.getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime, &searchEps);
		printf("Solution Found = %d\n", solutionFound);   
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
	}
	else
		printf("Solution does not exist\n");

	fflush(NULL);

    environment_navIntervallat.dumpStatesToFile();

    return bRet;
}


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
  
  if(atleast_one_collision){
	ROS_INFO("Collision !!\n");
	ROS_INFO("We will now consider %d more dynamic obstacles for the next iteration\n",obsCount);
  }
  else{
	ROS_INFO("No collision. We are done :)\n");
	}
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

int planAnytimeIntervallat(int argc, char *argv[], double eps)
{	


	int bRet = 0;
	double time_horizon_secs = 10;
	double total_allocated_time_secs = 30.0; 
	double SIPP_allocated_time_secs = 1; //in seconds (1200)
	bool bsearchuntilfirstsolution = false;
	bool in_collision = true;
	vector<vector<vector <double> > > dynObsTrajs;
	vector<double> dynObsRad;
	vector<SBPL_DynamicObstacle_t> dynObs;

	clock_t start_time,current_time;
	vector< vector< vector< vector<int> > > > globalTimelineMap;
	vector<bool> marked_obstacles;
        vector<bool> trajectory_indices;

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
	readDynObsTrajs(dynObsTrajs, dynObsRad, argv[3]);
	marked_obstacles.resize((int)dynObsTrajs.size());
	trajectory_indices.resize((int)dynObsTrajs.size());
	
	// Initialize all dynamic obstacles with trajectories limited to time horizon
	initializeDynamicObstacles(time_horizon_secs, dynObsTrajs, dynObsRad, marked_obstacles, dynObs);

        start_time = clock();


	while(in_collision == true && (clock()-start_time)<CLOCKS_PER_SEC*(total_allocated_time_secs-SIPP_allocated_time_secs)){
		
				    
	        EnvIntervalLat environment_navIntervallat_dynamic;
	
		if(!environment_navIntervallat_dynamic.InitializeEnv(argv[1], perimeterptsV, argv[2])){
		printf("ERROR: InitializeEnv failed\n");
		exit(1);
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

		    FILE* fSol = fopen("sol.txt", "w");
			vector<SBPL_4Dpt_t> xythetatimePath;
			environment_navIntervallat_dynamic.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
			printf("solution size=%d\n", (int)xythetatimePath.size());
			for(unsigned int i = 0; i < xythetatimePath.size(); i++) {
				fprintf(fSol, "%.3f %.3f %.3f %.3f %.3f\n", xythetatimePath.at(i).x, xythetatimePath.at(i).y, xythetatimePath.at(i).z, xythetatimePath.at(i).theta, xythetatimePath.at(i).t);
			}
		    fclose(fSol);
		    environment_navIntervallat_dynamic.PrintTimeStat(stdout);

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
				printf("Solution Found = %d\n", solutionFound);   
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
			}
			else
				printf("Solution does not exist\n");

			fflush(NULL);

		    	environment_navIntervallat_dynamic.dumpStatesToFile();
			
			in_collision = collisionCheck(xythetatimePath, dynObsTrajs, dynObsRad, robot_radius, marked_obstacles, dynObs);
			
		  solution_stateIDs_V.clear();
		  xythetatimePath.clear();

	}
	
    if(in_collision){
	SBPL_ERROR("The planner could not find a safe path within the allocated time");
	exit(1);
    }
    return bRet;
}

int main(int argc, char *argv[])
{
	
	if(argc < 2){
		PrintUsage(argv);
		exit(1);
	}
        //planIntervallat(argc, argv,1.0);
	planAnytimeIntervallat(argc,argv,3.0);
	return 0;
}





