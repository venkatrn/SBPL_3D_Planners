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

#define DRAW_MAP 0
#define DEFAULT_TEMPORAL_PADDING 100 //15

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0; //TODO-debugmax


//-----------------constructors/destructors-------------------------------
EnvDBubbleLattice::EnvDBubbleLattice()
{
  SBPL_PRINTF("u1\n");
	envDBubbleLatCfg.obsthresh = ENVDBUBBLELAT_DEFAULTOBSTHRESH;
	envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh = ENVDBUBBLELAT_DEFAULTDYNOBSTHRESH;
	envDBubbleLatCfg.cost_inscribed_thresh = envDBubbleLatCfg.obsthresh; //the value that pretty much makes it disabled
	envDBubbleLatCfg.cost_possibly_circumscribed_thresh = -1; //the value that pretty much makes it disabled

	grid2Dsearchfromstart = NULL;
	grid2Dsearchfromgoal = NULL;
	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;
	iteration = 0;

	envDBubbleLat.bInitialized = false;

	envDBubbleLatCfg.actionwidth = ENVDBUBBLELAT_DEFAULT_ACTIONWIDTH;
}

EnvDBubbleLattice::~EnvDBubbleLattice()
{
	if(grid2Dsearchfromstart != NULL)
		delete grid2Dsearchfromstart;
	grid2Dsearchfromstart = NULL;

	if(grid2Dsearchfromgoal != NULL)
		delete grid2Dsearchfromgoal;
	grid2Dsearchfromgoal = NULL;

  //free the environment
	for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++)
		delete [] envDBubbleLatCfg.Grid2D[x];
	delete [] envDBubbleLatCfg.Grid2D;

	for(int i=0; i<ENVDBUBBLELAT_THETADIRS; i++)
		delete [] envDBubbleLatCfg.ActionsV[i];  
	delete [] envDBubbleLatCfg.ActionsV;

	for(int  i=0; i<ENVDBUBBLELAT_THETADIRS; i++)
    envDBubbleLatCfg.PredActionsV[i].clear();
	delete [] envDBubbleLatCfg.PredActionsV;
}

EnvDBubbleLat::~EnvDBubbleLat(){
	for(unsigned int i=0; i<StateID2CoordTable.size(); i++)
    delete StateID2CoordTable[i];
	delete [] Coord2StateIDHashTable;
}

//---------------------------------------------------------------------


//-------------------problem specific and local functions---------------------


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

void EnvDBubbleLattice::ReadDynamicObstacles(FILE* fDynObs){
	char sTemp[1024], sTemp1[1024];
  int iTemp;

  SBPL_PRINTF("Reading Dynamic Obstacles...\n");

  //get the number of dynamic obstacles in the file
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "NumberOfDynamicObstacles:");
	if(strcmp(sTemp1, sTemp) != 0){
		SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
    throw new SBPL_Exception();
  }
	int numObs = atoi(sTemp);

  //for each dynamic obstacle
  for(int i=0; i < numObs; i++){

    //check that the ID matches i
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "DynamicObstacleID:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    iTemp = atoi(sTemp);
    if(iTemp != i){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %d got %d\n", i, iTemp);
      throw new SBPL_Exception();
    }
    SBPL_DynamicObstacle_t obs;

    //Read in the obstacle's radius
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "ObstacleRadius:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    obs.radius = atof(sTemp) + envDBubbleLatCfg.robotRadius;

    //read the number of trajectories for this obstacle
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "NumberOfTrajectories:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    int numTraj = atoi(sTemp);

    //for each trajectory
    double trajProbSum = 0;
    for(int j=0; j < numTraj; j++){
       
      //check that the ID matches j
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "TrajectoryID:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      iTemp = atoi(sTemp);
      if(iTemp != j){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %d got %d\n", j, iTemp);
        throw new SBPL_Exception();
      }
      SBPL_Trajectory_t traj;

      //read in this trajectory's probability
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "TrajectoryProbability:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      traj.prob = atof(sTemp);
      if(traj.prob < 0 || traj.prob > 1){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected TrajectoryProbability on the interval [0,1] but got %f\n", traj.prob);
        throw new SBPL_Exception();
      }
      trajProbSum += traj.prob;

      //read the number of intermediate points are given for the trajectory
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "NumberOfPoints:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      int numPoints = atoi(sTemp);

      //for each point
      int prev_t = 0;
      for(int k=0; k < numPoints; k++){
        //fill in the point
        SBPL_Traj_Pt_t pt;
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.x = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.y = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.t = CONTTIME2DISC(atof(sTemp),envDBubbleLatCfg.timeResolution);

        if(prev_t > pt.t && k != 0){
          SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
          SBPL_PRINTF("dynamic obstacle trajectory times can't decrease!\n");
          throw new SBPL_Exception();
        }
        prev_t = pt.t;

        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.std_dev = atof(sTemp);

        //store the point in the trajectory
        traj.points.push_back(pt);
      }

      //check if the obstacle should "disappear" after it has finished its trajectory
      //or if it sits in the configuration from the last frame of the trajectory forever
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "ObstacleExistsAfterTrajectory:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      traj.existsAfter = atoi(sTemp);
      if(traj.existsAfter != 0 && traj.existsAfter != 1){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("ObstacleExistsAfterTrajectory is a boolean and needs to be 0 or 1\n");
        throw new SBPL_Exception();
      }

      //store the trajectory in the dynamic obstacle
      obs.trajectories.push_back(traj);
    }

    //check that the trajectory probabilities sum to 1
    if(fabs(trajProbSum - 1.0) > ERR_EPS){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Probabilities for trajectories of dynamic obstacle %d sum to %f instead of 1\n", i, trajProbSum);
      throw new SBPL_Exception();
    }

    //store the dynamic obstacle into the dynamic obstacle vector
    dynamicObstacles.push_back(obs);

  }
/*
  for(int i=0; i<dynamicObstacles.size(); i++){
    SBPL_PRINTF("obs %d: radiusSquared=%f\n",i,dynamicObstacles[i].radiusSquared);
    for(int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      SBPL_PRINTF("  traj %d: prob=%f\n",j,dynamicObstacles[i].trajectories[j].prob);
      for(int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        SBPL_PRINTF("    point %d: x=%f y=%f t=%d std_dev=%f\n",k,p.x,p.y,p.t,p.std_dev);
      }
    }
  }
*/
  SBPL_PRINTF("Done Reading Dynamic Obstacles\n");
}

void EnvDBubbleLattice::InitializeBubbleMap(){
  SBPL_PRINTF("Initializing bubble map\n");
  //create a clean bubblemap for dynamic obstacles
  bubblemap.resize(envDBubbleLatCfg.EnvWidth_c);
  for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++){
    bubblemap[x].resize(envDBubbleLatCfg.EnvHeight_c);
  }

  double maxMovement = 0;
	for (unsigned int i = 0; i < envDBubbleLatCfg.mprimV.size(); i++){
    SBPL_4Dpt_t p = envDBubbleLatCfg.mprimV[i].intermptV.back();
    double temp = p.x*p.x + p.y*p.y;
    if(temp > maxMovement)
      maxMovement = temp;
  }
  envDBubbleLatCfg.maxMovement = sqrt(maxMovement);

  UpdateBubbleMap();
}

void EnvDBubbleLattice::UpdateBubbleMap(){
  SBPL_PRINTF("clearing bubble map\n");
  //clear the bubble map
  for(unsigned int x=0; x<bubblemap.size(); x++){
    for(unsigned int y=0; y<bubblemap[x].size(); y++){
      bubblemap[x][y].dynObs.clear();
      bubblemap[x][y].firstObsT = INFINITECOST;
      bubblemap[x][y].lastObsT = 0;
    }
  }

  SBPL_PRINTF("filling in new bubble map\n");
  int bubbleCount = 0;
  //fill in new bubble map
  for(unsigned int i=0; i<dynamicObstacles.size(); i++){
    for(unsigned int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      //TODO:Mike handle trajectory probabilities for now I'll assume all probabilities are 1
      for(unsigned int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        int bubble_cell_rad = (int)ceil((dynamicObstacles[i].radius + p.std_dev*3 + envDBubbleLatCfg.maxMovement)/envDBubbleLatCfg.cellsize_m);
        int bubble_cell_inner_rad = (int)ceil((dynamicObstacles[i].radius + p.std_dev*3)/envDBubbleLatCfg.cellsize_m);
        FillInBubble(CONTXY2DISC(p.x, envDBubbleLatCfg.cellsize_m), CONTXY2DISC(p.y, envDBubbleLatCfg.cellsize_m), p.t, &(dynamicObstacles[i]), bubble_cell_inner_rad, bubbleCount, true);
        FillInBubble(CONTXY2DISC(p.x, envDBubbleLatCfg.cellsize_m), CONTXY2DISC(p.y, envDBubbleLatCfg.cellsize_m), p.t, &(dynamicObstacles[i]), bubble_cell_rad, bubbleCount, false);
        bubbleCount++;
      }
    }
  }

  bubble4Dactive.resize(bubbleCount);
  for (int i=0; i < bubbleCount; i++)
    bubble4Dactive[i] = false;
  SBPL_PRINTF("%d bubbles\n", bubbleCount);

#if DRAW_MAP  
  SBPL_PRINTF("BubbleMap with last obstacle time in each cell\n");
  //draw bubble map
	for(int y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++){
		for(int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++){
      SBPL_PRINTF("%d ",bubblemap[x][y].lastObsT);
		}
    SBPL_PRINTF("\n");
  }
  SBPL_PRINTF("BubbleMap with collision intervals in each cell for obstacle 1\n");
  //draw bubble map
	for(int y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++){
		for(int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++){
      if(bubblemap[x][y].dynObs.empty())
        SBPL_PRINTF("(*,*) ");
      else if(bubblemap[x][y].dynObs[0].collision_startt == INFINITECOST)
        SBPL_PRINTF("(*,*) ");
      else
        SBPL_PRINTF("(%d,%d) ",bubblemap[x][y].dynObs[0].collision_startt, bubblemap[x][y].dynObs[0].collision_endt);
		}
    SBPL_PRINTF("\n");
  }
#endif
  
  SBPL_PRINTF("done updating bubble map\n");
}

//Bresenham Circle Algorithm for finding edge of the circle so I can mark all cells inside
void EnvDBubbleLattice::FillInBubble(int CenterX, int CenterY, int T, SBPL_DynamicObstacle_t* obs, int rad, int ID, bool isInnerBubble){
  //SBPL_PRINTF("begin circle %d at (%d,%d,%d)  with radius %d\n",ID,CenterX,CenterY,T,rad);
  int d = 3 - 2*rad;
  int x = 0;
  int y = rad;

  while(x <= y){
    //mark pixels in 4 columns
    FillInBubbleColumn(CenterX-y, CenterY+x, CenterY-x, T, obs, ID, isInnerBubble);
    FillInBubbleColumn(CenterX-x, CenterY+y, CenterY-y, T, obs, ID, isInnerBubble);
    FillInBubbleColumn(CenterX+x, CenterY+y, CenterY-y, T, obs, ID, isInnerBubble);
    FillInBubbleColumn(CenterX+y, CenterY+x, CenterY-x, T, obs, ID, isInnerBubble);
    if(d <= 0)
      d += 4*x + 6;
    else{
      d += 4*(x-y) + 10;
      y--;
    }
    x++;
  }
  //SBPL_PRINTF("end circle\n");
}

void EnvDBubbleLattice::FillInBubbleColumn(int x, int topY, int botY, int T, SBPL_DynamicObstacle_t* obs, int ID, bool isInnerBubble){
  //SBPL_PRINTF("begin column %d from %d to %d\n", x, topY, botY);
  //return if column is outside map
  if(x < 0 || x >= (int)bubblemap.size())
    return;
  //snap topY and botY to edge of map if they are off of it
  if(topY >= (int)bubblemap[x].size())
    topY = bubblemap[x].size()-1;
  if(botY < 0)
    botY = 0;
  //return if column is outside map
  if(topY < 0 || botY >= (int)bubblemap[x].size())
    return;

  //SBPL_PRINTF("fill in top\n");
  //fill in top of column
  FillInBubbleCell(x, topY, T, obs, ID, isInnerBubble);
  
  //SBPL_PRINTF("fill in bottom\n");
  //fill in bottom of column
  FillInBubbleCell(x, botY, T, obs, ID, isInnerBubble);

  //SBPL_PRINTF("check if there is more column\n");
  //check if the rest of this column has already been filled in and if so we are done

  if(topY-botY <= 1)
    return;

  if(bubblemap[x][topY-1].dynObs.size() == 0 || bubblemap[x][topY-1].dynObs.back().ID.back() != ID){
    //otherwise fill in the column
    for(int y = topY-1; y>botY; y--){
      FillInBubbleCell(x, y, T, obs, ID, isInnerBubble);
    }
  }

  if(bubblemap[x][botY+1].dynObs.size() == 0 || bubblemap[x][botY+1].dynObs.back().ID.back() != ID){
    //otherwise fill in the column
    for(int y = botY+1; y<topY; y++){
      FillInBubbleCell(x, y, T, obs, ID, isInnerBubble);
    }
  }
  //SBPL_PRINTF("end column\n");
}

void EnvDBubbleLattice::FillInBubbleCell(int x, int y, int T, SBPL_DynamicObstacle_t* obs, int ID, bool isInnerBubble){
  int minT = T - temporal_padding;
  int maxT = T + temporal_padding;
  if(bubblemap[x][y].dynObs.size() == 0 || bubblemap[x][y].dynObs.back().obs != obs){
    envDBubbleLat_BubbleCellObs_t cell;
    cell.startt = minT;
    cell.endt = maxT;
    cell.obs = obs;
    cell.ID.push_back(ID);
    if(isInnerBubble){
      cell.collision_startt = minT;
      cell.collision_endt = maxT;
    }
    else{
      cell.collision_startt = INFINITECOST;
      cell.collision_endt = -INFINITECOST;
    }
    bubblemap[x][y].dynObs.push_back(cell);
  }
  else{
    if(minT < bubblemap[x][y].dynObs.back().startt)
      bubblemap[x][y].dynObs.back().startt = minT;
    if(maxT > bubblemap[x][y].dynObs.back().endt)
      bubblemap[x][y].dynObs.back().endt = maxT;
    if(isInnerBubble){
      if(minT < bubblemap[x][y].dynObs.back().collision_startt)
        bubblemap[x][y].dynObs.back().collision_startt = minT;
      if(maxT > bubblemap[x][y].dynObs.back().collision_endt)
        bubblemap[x][y].dynObs.back().collision_endt = maxT;
    }
    if(ID != bubblemap[x][y].dynObs.back().ID.back())
      bubblemap[x][y].dynObs.back().ID.push_back(ID);
  }
  if(maxT > bubblemap[x][y].lastObsT)
    bubblemap[x][y].lastObsT = maxT;
  if(minT < bubblemap[x][y].firstObsT)
    bubblemap[x][y].firstObsT = minT;
}

int EnvDBubbleLattice::getNumBubbles(){
  return bubble4Dactive.size();
}

void EnvDBubbleLattice::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta, int startTime,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double timeResolution, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
          const vector<sbpl_2Dpt_t> & robot_perimeterV) {
  envDBubbleLatCfg.EnvWidth_c = width;
  envDBubbleLatCfg.EnvHeight_c = height;
  envDBubbleLatCfg.StartX_c = startx;
  envDBubbleLatCfg.StartY_c = starty;
  envDBubbleLatCfg.StartTheta = starttheta;
  envDBubbleLatCfg.StartTime = startTime;
 
  if(envDBubbleLatCfg.StartX_c < 0 || envDBubbleLatCfg.StartX_c >= envDBubbleLatCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(envDBubbleLatCfg.StartY_c < 0 || envDBubbleLatCfg.StartY_c >= envDBubbleLatCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(envDBubbleLatCfg.StartTheta < 0 || envDBubbleLatCfg.StartTheta >= ENVDBUBBLELAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
    throw new SBPL_Exception();
  }
  
  envDBubbleLatCfg.EndX_c = goalx;
  envDBubbleLatCfg.EndY_c = goaly;
  envDBubbleLatCfg.EndTheta = goaltheta;

  if(envDBubbleLatCfg.EndX_c < 0 || envDBubbleLatCfg.EndX_c >= envDBubbleLatCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(envDBubbleLatCfg.EndY_c < 0 || envDBubbleLatCfg.EndY_c >= envDBubbleLatCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(envDBubbleLatCfg.EndTheta < 0 || envDBubbleLatCfg.EndTheta >= ENVDBUBBLELAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
    throw new SBPL_Exception();
  }

  envDBubbleLatCfg.FootprintPolygon = robot_perimeterV;

  envDBubbleLatCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  envDBubbleLatCfg.cellsize_m = cellsize_m;
  envDBubbleLatCfg.timeResolution = timeResolution;
  envDBubbleLatCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  envDBubbleLatCfg.Grid2D = new unsigned char* [envDBubbleLatCfg.EnvWidth_c];
  for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++) {
    envDBubbleLatCfg.Grid2D[x] = new unsigned char [envDBubbleLatCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++) {
      for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++) {
	envDBubbleLatCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++) {
      for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++) {
			envDBubbleLatCfg.Grid2D[x][y] = mapdata[x+y*width];
      }
    }
  }
}

void EnvDBubbleLattice::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  envDBubbleLatCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;
	int x, y;

	//discretization(cells)
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "discretization(cells):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.EnvWidth_c = atoi(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.EnvHeight_c = atoi(sTemp);

	//obsthresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "obsthresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.obsthresh = atoi(sTemp);
	SBPL_PRINTF("obsthresh = %d\n", envDBubbleLatCfg.obsthresh);

	//cost_inscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cost_inscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.cost_inscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_inscribed_thresh = %d\n", envDBubbleLatCfg.cost_inscribed_thresh);


	//cost_possibly_circumscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", envDBubbleLatCfg.cost_possibly_circumscribed_thresh);

	//dynamic_obstacle_collision_cost_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "dynamic_obstacle_collision_cost_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh = atoi(sTemp);
	SBPL_PRINTF("dynamic_obstacle_collision_cost_thresh = %d\n", envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh);
	
	//cellsize
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cellsize(meters):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.cellsize_m = atof(sTemp);
	
	//timeResolution
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "timeResolution(seconds):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.timeResolution = atof(sTemp);

  //temporal_padding
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "temporal_padding(seconds):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	temporal_padding = (int)ceil(atof(sTemp)/envDBubbleLatCfg.timeResolution);
	SBPL_PRINTF("temporal_padding = %d\n", temporal_padding);

	//speeds
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "nominalvel(mpersecs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.nominalvel_mpersecs = atof(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.StartX_c = CONTXY2DISC(atof(sTemp),envDBubbleLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.StartY_c = CONTXY2DISC(atof(sTemp),envDBubbleLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.StartTheta = ContTheta2Disc(atof(sTemp), ENVDBUBBLELAT_THETADIRS);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.StartTime = CONTTIME2DISC(atof(sTemp),envDBubbleLatCfg.timeResolution);


	if(envDBubbleLatCfg.StartX_c < 0 || envDBubbleLatCfg.StartX_c >= envDBubbleLatCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(envDBubbleLatCfg.StartY_c < 0 || envDBubbleLatCfg.StartY_c >= envDBubbleLatCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(envDBubbleLatCfg.StartTheta < 0 || envDBubbleLatCfg.StartTheta >= ENVDBUBBLELAT_THETADIRS) {
		SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
		throw new SBPL_Exception();
	}

	//end(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.EndX_c = CONTXY2DISC(atof(sTemp),envDBubbleLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.EndY_c = CONTXY2DISC(atof(sTemp),envDBubbleLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.EndTheta = ContTheta2Disc(atof(sTemp), ENVDBUBBLELAT_THETADIRS);;

	if(envDBubbleLatCfg.EndX_c < 0 || envDBubbleLatCfg.EndX_c >= envDBubbleLatCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(envDBubbleLatCfg.EndY_c < 0 || envDBubbleLatCfg.EndY_c >= envDBubbleLatCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(envDBubbleLatCfg.EndTheta < 0 || envDBubbleLatCfg.EndTheta >= ENVDBUBBLELAT_THETADIRS) {
		SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
		throw new SBPL_Exception();
	}


	//allocate the 2D environment
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envDBubbleLatCfg.Grid2D = new unsigned char* [envDBubbleLatCfg.EnvWidth_c];
	for (x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++)
	{
		envDBubbleLatCfg.Grid2D[x] = new unsigned char [envDBubbleLatCfg.EnvHeight_c];
	}

	//environment:
	for (y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++){
		for (x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				SBPL_ERROR("ERROR: incorrect format of config file (%d,%d)\n",y,x);
				throw new SBPL_Exception();
			}
			envDBubbleLatCfg.Grid2D[x][y] = dTemp;
#if DRAW_MAP
      SBPL_PRINTF("%d ",dTemp);
#endif
		}
#if DRAW_MAP
    SBPL_PRINTF("\n");
#endif
  }

}

int EnvDBubbleLattice::ReadinCell(SBPL_4Dcell_t* cell, char* fIn)
{
  char* temp;
  char* sptr;
  
  if((temp = strtok_r(fIn, " ", &sptr)) == NULL)
    return FAIL;
  cell->x = atoi(temp);
  
  if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
    return FAIL;
  cell->y = atoi(temp);
  
  if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
    return FAIL;
  cell->theta = atoi(temp);
  //normalize the angle
  cell->theta = NORMALIZEDISCTHETA(cell->theta, ENVDBUBBLELAT_THETADIRS);
  
  //This is an optional param
  if((temp = strtok_r(NULL, " ", &sptr)) != NULL){
    cell->t = atoi(temp);
    return SUCCESS_WITH_TIME;
  }

  return SUCCESS_NO_TIME;
}

int EnvDBubbleLattice::ReadinPose(SBPL_4Dpt_t* pose, char* fIn)
{
  char* temp;
  char* sptr;

	if((temp = strtok_r(fIn, " ", &sptr)) == NULL)
	   return FAIL;
	pose->x = atof(temp);

	if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
	   return FAIL;
	pose->y = atof(temp);

	if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
	   return FAIL;
	pose->theta = atof(temp);
	pose->theta = normalizeAngle(pose->theta);

  //This is an optional param
	if((temp = strtok_r(NULL, " ", &sptr)) != NULL){
    pose->t = atof(temp);
    return SUCCESS_WITH_TIME;
  }

	return SUCCESS_NO_TIME;
}

bool EnvDBubbleLattice::ReadinMotionPrimitive(SBPL_xythetatimebubble_mprimitive* pMotPrim, FILE* fIn, bool &isWaitMotion)
{
  int bufSize = 1024;
  char sTemp[bufSize];
	int dTemp;
  char sExpected[bufSize];
  int numofIntermPoses;
  bool timeGiven = true;
  int timeGivenTemp;

  //read in actionID
  strcpy(sExpected, "primID:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
    return false;

  //read in start angle
  strcpy(sExpected, "startangle_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) == 0)
  {
    SBPL_ERROR("ERROR reading startangle\n");
    return false;	
  }
  pMotPrim->starttheta_c = dTemp;
   
  //read in end pose
  strcpy(sExpected, "endpose_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }

  if(fgets(sTemp, bufSize, fIn) == NULL)
    return false;
  timeGivenTemp = ReadinCell(&pMotPrim->endcell, sTemp);
  if(timeGivenTemp == FAIL){
    SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
    return false;
  }
  else if(timeGivenTemp == SUCCESS_NO_TIME)
    timeGiven = false;
  
 
  //read in action cost
  strcpy(sExpected, "additionalactioncostmult:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) != 1)
    return false;
	pMotPrim->additionalactioncostmult = dTemp;
    
  //read in intermediate poses
  strcpy(sExpected, "intermediateposes:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
    return false;

	//all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done 
	//after the action is rotated by initial orientation
  if(fgets(sTemp, bufSize, fIn) == NULL)//this gets rid of the new line character from the previous line
    return false;
  isWaitMotion = true;
  for(int i = 0; i < numofIntermPoses; i++){
    SBPL_4Dpt_t intermpose;
    if(fgets(sTemp, bufSize, fIn) == NULL)
      return false;
    timeGivenTemp = ReadinPose(&intermpose, sTemp);
    if(timeGivenTemp == FAIL){
      SBPL_ERROR("ERROR: failed to read in intermediate pose %d\n",i);
      return false;
    }
    else if(timeGivenTemp == SUCCESS_NO_TIME)
      timeGiven = false;

    isWaitMotion = isWaitMotion && intermpose.x == 0 && intermpose.y == 0 && (i==0 || intermpose.theta == pMotPrim->intermptV.back().theta);
    pMotPrim->intermptV.push_back(intermpose);
  }

  //compute time intervals if they were not given
  if(!timeGiven){
    double x = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
    double y = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
    double theta = fabs(computeMinUnsignedAngleDiff(pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta,
                                                    pMotPrim->intermptV[0].theta));
    double timeFromTranslation = sqrt(x*x + y*y)/envDBubbleLatCfg.nominalvel_mpersecs;
    double timeFromRotation = theta/(PI_CONST/4.0)*envDBubbleLatCfg.timetoturn45degsinplace_secs;
    double t = __max(timeFromTranslation, timeFromRotation);
    if(t == 0)
      t = envDBubbleLatCfg.timeResolution;

    pMotPrim->endcell.t = (int)(t/envDBubbleLatCfg.timeResolution);
    double step = t/(pMotPrim->intermptV.size()-1);
    double temp = 0;
    for(unsigned int i = 0; i < pMotPrim->intermptV.size(); i++){
      pMotPrim->intermptV[i].t = temp;
      temp += step;
    }
  }
  //TODO:Mike if times were given at least make sure they are increasing...

	//check that the last pose corresponds correctly to the last pose
	SBPL_4Dpt_t sourcepose;
	sourcepose.x = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
	sourcepose.y = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
	sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, ENVDBUBBLELAT_THETADIRS);
  sourcepose.t = 0;
	double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
	double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
	double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;				
	double mp_endt = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].t;				
	int endx_c = CONTXY2DISC(mp_endx_m, envDBubbleLatCfg.cellsize_m);
	int endy_c = CONTXY2DISC(mp_endy_m, envDBubbleLatCfg.cellsize_m);
	int endtheta_c = ContTheta2Disc(mp_endtheta_rad, ENVDBUBBLELAT_THETADIRS);
	int endt_c = CONTTIME2DISC(mp_endt, envDBubbleLatCfg.timeResolution);
	if(endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta || endt_c != pMotPrim->endcell.t)
	{	
		SBPL_ERROR("ERROR: incorrect primitive %d with startangle=%d last interm point %f %f %f %f does not match end pose %d %d %d %d\n", 
    pMotPrim->motprimID, pMotPrim->starttheta_c,
    pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].t,
    pMotPrim->endcell.x, pMotPrim->endcell.y,pMotPrim->endcell.theta,pMotPrim->endcell.t);	
    return false;
	}
  
  return true;
}



bool EnvDBubbleLattice::ReadMotionPrimitives(FILE* fMotPrims)
{
  char sTemp[1024], sExpected[1024];
  float fTemp;
  int dTemp;
  int totalNumofActions = 0;

  SBPL_PRINTF("Reading in motion primitives...");

  //read in the resolution
  strcpy(sExpected, "resolution_m:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%f", &fTemp) == 0)
    return false;
  if(fabs(fTemp-envDBubbleLatCfg.cellsize_m) > ERR_EPS){
    SBPL_ERROR("ERROR: invalid grid resolution %f (instead of %f) in the dynamics file\n", 
        fTemp, envDBubbleLatCfg.cellsize_m);
    return false;
  }

  //read in time resolution
  strcpy(sExpected, "timeResolution:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%f", &fTemp) == 0)
    return false;
  if(fabs(fTemp-envDBubbleLatCfg.timeResolution) > ERR_EPS){
    SBPL_ERROR("ERROR: invalid time resolution %f (instead of %f) in the dynamics file\n", 
        fTemp, envDBubbleLatCfg.timeResolution);
    return false;
  }

  //read in the angular resolution
  strcpy(sExpected, "numberofangles:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%d", &dTemp) == 0)
    return false;
  if(dTemp != ENVDBUBBLELAT_THETADIRS){
    SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", 
        dTemp, ENVDBUBBLELAT_THETADIRS);
    return false;
  }


  //read in the total number of actions
  strcpy(sExpected, "totalnumberofprimitives:");
  if(fscanf(fMotPrims, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
    return false;
  }

  vector<bool> waitMotFlags;
  waitMotFlags.resize(ENVDBUBBLELAT_THETADIRS);
  for(int i = 0; i < ENVDBUBBLELAT_THETADIRS; i++)
    waitMotFlags[i] = false;
  vector<SBPL_xythetatimebubble_mprimitive> waitMots;
  for(int i = 0; i < totalNumofActions; i++){
    SBPL_xythetatimebubble_mprimitive motprim;

    bool isWaitMotion;
    if(EnvDBubbleLattice::ReadinMotionPrimitive(&motprim, fMotPrims, isWaitMotion) == false)
      return false;

    if(isWaitMotion){
      if(!waitMotFlags[motprim.starttheta_c]){
        waitMotFlags[motprim.starttheta_c] = true;
        waitMots.push_back(motprim);
      }
      else{
        SBPL_ERROR("ERROR: only one wait in place motion is allowed per theta direction!\n");
        return false;
      }
    }
    else
      envDBubbleLatCfg.mprimV.push_back(motprim);

  }
    
  //put wait motion at the end of the list if there is one, if there isn't create one
  if(waitMots.empty()){
    //they didn't give us a wait motion so add one
    SBPL_PRINTF("Adding a motion primitive since one was not provided...\n");
    int baseID = envDBubbleLatCfg.mprimV.back().motprimID + 1;
    for(int tind = 0; tind < ENVDBUBBLELAT_THETADIRS; tind++){
      SBPL_xythetatimebubble_mprimitive motprim;
      motprim.motprimID = baseID + tind;
      motprim.starttheta_c = tind;
      motprim.endcell.x = 0;
      motprim.endcell.y = 0;
      motprim.endcell.theta = tind;
      motprim.endcell.t = 1;
      motprim.additionalactioncostmult = 1;

      SBPL_4Dpt_t intermpose1;
      intermpose1.x = 0;
      intermpose1.y = 0;
      intermpose1.theta = DiscTheta2Cont(tind, ENVDBUBBLELAT_THETADIRS);
      intermpose1.t = 0;
      motprim.intermptV.push_back(intermpose1);

      SBPL_4Dpt_t intermpose2;
      intermpose2.x = 0;
      intermpose2.y = 0;
      intermpose2.theta = DiscTheta2Cont(tind, ENVDBUBBLELAT_THETADIRS);
      intermpose2.t = envDBubbleLatCfg.timeResolution;
      motprim.intermptV.push_back(intermpose2);
      
      envDBubbleLatCfg.mprimV.push_back(motprim);
    }
  }
  else{
    //make sure wait is at the end of the vector
    if(waitMots.size() != ENVDBUBBLELAT_THETADIRS){
      SBPL_PRINTF("Error: wait actions only provided for some theta directions\n");
      return false;
    } 
    for(unsigned int i=0; i<waitMots.size(); i++)
      envDBubbleLatCfg.mprimV.push_back(waitMots[i]);
  }
  SBPL_PRINTF("done ");

  return true;
}


void EnvDBubbleLattice::ComputeReplanningDataforAction(envDBubbleLatAction_t* action)
{
	int j;

	//iterate over all the cells involved in the action
	SBPL_4Dcell_t startcell4d, endcell4d;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
	{

		//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
		startcell4d.theta = action->starttheta;
		startcell4d.x = - action->intersectingcellsV.at(i).x;
		startcell4d.y = - action->intersectingcellsV.at(i).y;
    startcell4d.t = - action->intersectingcellsV.at(i).t;

		//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
		endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVDBUBBLELAT_THETADIRS); 
		endcell4d.x = startcell4d.x + action->dX; 
		endcell4d.y = startcell4d.y + action->dY;
		endcell4d.t = startcell4d.t + action->dT;

		//store the cells if not already there
		for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
		{
			if(affectedsuccstatesV.at(j) == endcell4d)
				break;
		}
		if (j == (int)affectedsuccstatesV.size())
			affectedsuccstatesV.push_back(endcell4d);

		for(j = 0; j < (int)affectedpredstatesV.size(); j++)
		{
			if(affectedpredstatesV.at(j) == startcell4d)
				break;
		}
		if (j == (int)affectedpredstatesV.size())
			affectedpredstatesV.push_back(startcell4d);

  }//over intersecting cells

	

	//add the centers since with h2d we are using these in cost computations
	//---intersecting cell = origin
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell4d.theta = action->starttheta;
	startcell4d.x = - 0;
	startcell4d.y = - 0;
	startcell4d.t = - 0;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVDBUBBLELAT_THETADIRS); 
	endcell4d.x = startcell4d.x + action->dX; 
	endcell4d.y = startcell4d.y + action->dY;
	endcell4d.t = startcell4d.y + action->dT;

	//store the cells if not already there
	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell4d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell4d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell4d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell4d);


	//---intersecting cell = outcome state
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell4d.theta = action->starttheta;
	startcell4d.x = - action->dX;
	startcell4d.y = - action->dY;
	startcell4d.t = - action->dT;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVDBUBBLELAT_THETADIRS); 
	endcell4d.x = startcell4d.x + action->dX; 
	endcell4d.y = startcell4d.y + action->dY;
	endcell4d.t = startcell4d.t + action->dT;

	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell4d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell4d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell4d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell4d);


}


//computes all the 4D states whose outgoing actions are potentially affected when cell (0,0) changes its status
//it also does the same for the 4D states whose incoming actions are potentially affected when cell (0,0) changes its status
void EnvDBubbleLattice::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < ENVDBUBBLELAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < envDBubbleLatCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&envDBubbleLatCfg.ActionsV[tind][aind]);
		}
	}
}

//here motionprimitivevector contains actions only for 0 angle
void EnvDBubbleLattice::PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetatimebubble_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
	envDBubbleLatCfg.ActionsV = new envDBubbleLatAction_t* [ENVDBUBBLELAT_THETADIRS];
	envDBubbleLatCfg.PredActionsV = new vector<envDBubbleLatAction_t*> [ENVDBUBBLELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	//iterate over source angles
	for(int tind = 0; tind < ENVDBUBBLELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, ENVDBUBBLELAT_THETADIRS);
		envDBubbleLatCfg.ActionsV[tind] = new envDBubbleLatAction_t[motionprimitiveV->size()];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVDBUBBLELAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, envDBubbleLatCfg.timeResolution);

		//iterate over motion primitives
		for(size_t aind = 0; aind < motionprimitiveV->size(); aind++)
		{
			envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;
			double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].x;
			double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].y;
			double mp_endtheta_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].theta;
			double mp_endt = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].t;
			
			double endx = sourcepose.x + (mp_endx_m*cos(sourcepose.theta) - mp_endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (mp_endx_m*sin(sourcepose.theta) + mp_endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, envDBubbleLatCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, envDBubbleLatCfg.cellsize_m);

			
			envDBubbleLatCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad+sourcepose.theta, ENVDBUBBLELAT_THETADIRS);
			envDBubbleLatCfg.ActionsV[tind][aind].dX = endx_c;
			envDBubbleLatCfg.ActionsV[tind][aind].dY = endy_c;
			envDBubbleLatCfg.ActionsV[tind][aind].dT = CONTTIME2DISC(mp_endt + sourcepose.t, envDBubbleLatCfg.timeResolution);
      /*
			if(envDBubbleLatCfg.ActionsV[tind][aind].dY != 0 || envDBubbleLatCfg.ActionsV[tind][aind].dX != 0) //if we translate
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.cellsize_m/envDBubbleLatCfg.nominalvel_mpersecs*
								sqrt((double)(envDBubbleLatCfg.ActionsV[tind][aind].dX*envDBubbleLatCfg.ActionsV[tind][aind].dX + 
								envDBubbleLatCfg.ActionsV[tind][aind].dY*envDBubbleLatCfg.ActionsV[tind][aind].dY))));
			else if(envDBubbleLatCfg.ActionsV[tind][aind].starttheta != envDBubbleLatCfg.ActionsV[tind][aind].endtheta)//else if we turn in place
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*
						envDBubbleLatCfg.timetoturn45degsinplace_secs*fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad,0))/(PI_CONST/4.0));
      else //else (we stand still)
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.ActionsV[tind][aind].dT*envDBubbleLatCfg.timeResolution);
				//envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution));
      */

      envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.ActionsV[tind][aind].dT*envDBubbleLatCfg.timeResolution);

			//compute and store interm points as well as intersecting cells
			envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
			envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.clear();
			SBPL_4Dcell_t previnterm4Dcell;
			previnterm4Dcell.theta = previnterm4Dcell.x = previnterm4Dcell.y = previnterm4Dcell.t = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++)
			{
				SBPL_4Dpt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];
		
				//rotate it appropriately
				double rotx = intermpt.x*cos(sourcepose.theta) - intermpt.y*sin(sourcepose.theta);
				double roty = intermpt.x*sin(sourcepose.theta) + intermpt.y*cos(sourcepose.theta);
				intermpt.x = rotx;
				intermpt.y = roty;
				intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);
        //don't need to set time since the intermpt.t is already correct since sourcepose.t is always 0

				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				envDBubbleLatCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);

				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, envDBubbleLatCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, envDBubbleLatCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, ENVDBUBBLELAT_THETADIRS); 
				interm4Dcell.t = CONTXY2DISC(pose.t, envDBubbleLatCfg.timeResolution);
				if(envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;

			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f %.2f)\n",
				tind, aind, 			
				envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.ActionsV[tind][aind].dT,
				envDBubbleLatCfg.ActionsV[tind][aind].endtheta, sourcepose.theta*180/PI_CONST, 
				envDBubbleLatCfg.ActionsV[tind][aind].intermptV[envDBubbleLatCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				envDBubbleLatCfg.ActionsV[tind][aind].cost,
				mp_endx_m, mp_endy_m, mp_endt, mp_endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
			 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));

		}
	}

	//set number of actions
	envDBubbleLatCfg.actionwidth = motionprimitiveV->size();


	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");


}


//here motionprimitivevector contains actions for all angles
void EnvDBubbleLattice::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetatimebubble_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
	envDBubbleLatCfg.ActionsV = new envDBubbleLatAction_t* [ENVDBUBBLELAT_THETADIRS];
	envDBubbleLatCfg.PredActionsV = new vector<envDBubbleLatAction_t*> [ENVDBUBBLELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	if(motionprimitiveV->size()%ENVDBUBBLELAT_THETADIRS != 0)
	{
		SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
		throw new SBPL_Exception();
	}

	envDBubbleLatCfg.actionwidth = ((int)motionprimitiveV->size())/ENVDBUBBLELAT_THETADIRS;

	//iterate over source angles
	int maxnumofactions = 0;
	for(int tind = 0; tind < ENVDBUBBLELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, ENVDBUBBLELAT_THETADIRS);

		envDBubbleLatCfg.ActionsV[tind] = new envDBubbleLatAction_t[envDBubbleLatCfg.actionwidth];  

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVDBUBBLELAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, envDBubbleLatCfg.timeResolution);


		//iterate over motion primitives
		int numofactions = 0;
		int aind = -1;
		for(int mind = 0; mind < (int)motionprimitiveV->size(); mind++)
		{
			//find a motion primitive for this angle
			if(motionprimitiveV->at(mind).starttheta_c != tind)
				continue;
			
			aind++;
			numofactions++;

			//start angle
			envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;

			//compute dislocation
			envDBubbleLatCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
			envDBubbleLatCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
			envDBubbleLatCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;
			envDBubbleLatCfg.ActionsV[tind][aind].dT = motionprimitiveV->at(mind).endcell.t;

      /*
			//compute cost (if translating)
			if(envDBubbleLatCfg.ActionsV[tind][aind].dY != 0 || envDBubbleLatCfg.ActionsV[tind][aind].dX != 0)
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.cellsize_m/envDBubbleLatCfg.nominalvel_mpersecs*
								sqrt((double)(envDBubbleLatCfg.ActionsV[tind][aind].dX*envDBubbleLatCfg.ActionsV[tind][aind].dX + 
								envDBubbleLatCfg.ActionsV[tind][aind].dY*envDBubbleLatCfg.ActionsV[tind][aind].dY))));
			else if(envDBubbleLatCfg.ActionsV[tind][aind].starttheta != envDBubbleLatCfg.ActionsV[tind][aind].endtheta)//cost (if turning in place)
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*
						envDBubbleLatCfg.timetoturn45degsinplace_secs*
						fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS),
														DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].starttheta, ENVDBUBBLELAT_THETADIRS)))/(PI_CONST/4.0));
      else //cost (if standing still)
				envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.ActionsV[tind][aind].dT*envDBubbleLatCfg.timeResolution);
				//envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution));
      */
      envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.ActionsV[tind][aind].dT*envDBubbleLatCfg.timeResolution);
      //SBPL_PRINTF("cost=%d\n",envDBubbleLatCfg.ActionsV[tind][aind].cost);

			//use any additional cost multiplier
			//envDBubbleLatCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

			//compute and store interm points as well as intersecting cells
			envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
			envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.clear();
			SBPL_4Dcell_t previnterm4Dcell;
			previnterm4Dcell.theta = 0; previnterm4Dcell.x = 0; previnterm4Dcell.y = previnterm4Dcell.t = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++)
			{
				SBPL_4Dpt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
		
				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				envDBubbleLatCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);
			
				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, envDBubbleLatCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, envDBubbleLatCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, ENVDBUBBLELAT_THETADIRS); 
				interm4Dcell.t = CONTTIME2DISC(pose.t, envDBubbleLatCfg.timeResolution);
				if(envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					envDBubbleLatCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;
			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprimID %d: %d %d %d %d)\n",
				tind, aind, 			
				envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.ActionsV[tind][aind].dT,
				envDBubbleLatCfg.ActionsV[tind][aind].endtheta, 
				envDBubbleLatCfg.ActionsV[tind][aind].intermptV[0].theta*180/PI_CONST, 
				envDBubbleLatCfg.ActionsV[tind][aind].intermptV[envDBubbleLatCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				envDBubbleLatCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(mind).motprimID, 
				motionprimitiveV->at(mind).endcell.x, motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.t, motionprimitiveV->at(mind).endcell.theta);
#endif

			//add to the list of backward actions
			int targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
			 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));

		}

		if(maxnumofactions < numofactions)
			maxnumofactions = numofactions;
	}



	//at this point we don't allow nonuniform number of actions
	if(motionprimitiveV->size() != (size_t)(ENVDBUBBLELAT_THETADIRS*maxnumofactions))
	{
		SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
				maxnumofactions, (int)motionprimitiveV->size(), ENVDBUBBLELAT_THETADIRS);
		throw new SBPL_Exception();
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");

}

void EnvDBubbleLattice::PrecomputeActions()
{

	//construct list of actions
	SBPL_PRINTF("Pre-computing action data using the motion primitives for a 4D kinematic planning...\n");
	envDBubbleLatCfg.ActionsV = new envDBubbleLatAction_t* [ENVDBUBBLELAT_THETADIRS];
	envDBubbleLatCfg.PredActionsV = new vector<envDBubbleLatAction_t*> [ENVDBUBBLELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < ENVDBUBBLELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("processing angle %d\n", tind);
		envDBubbleLatCfg.ActionsV[tind] = new envDBubbleLatAction_t[envDBubbleLatCfg.actionwidth];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envDBubbleLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVDBUBBLELAT_THETADIRS);
		sourcepose.t = DISCTIME2CONT(0, envDBubbleLatCfg.timeResolution);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;
			envDBubbleLatCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1)%ENVDBUBBLELAT_THETADIRS; //-1,0,1
			double angle = DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS);
			envDBubbleLatCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			envDBubbleLatCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
      //TODO:Mike we are ignoring the turn time in dT and in the cost, it is easily computable here 
      //and we know the robot rotates before moving so it seems like it should be added....
			envDBubbleLatCfg.ActionsV[tind][aind].dT = (int)(ceil(envDBubbleLatCfg.cellsize_m/envDBubbleLatCfg.nominalvel_mpersecs*sqrt((double)(envDBubbleLatCfg.ActionsV[tind][aind].dX*envDBubbleLatCfg.ActionsV[tind][aind].dX + envDBubbleLatCfg.ActionsV[tind][aind].dY*envDBubbleLatCfg.ActionsV[tind][aind].dY))));
      envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.cellsize_m/envDBubbleLatCfg.nominalvel_mpersecs*sqrt((double)(envDBubbleLatCfg.ActionsV[tind][aind].dX*envDBubbleLatCfg.ActionsV[tind][aind].dX + envDBubbleLatCfg.ActionsV[tind][aind].dY*envDBubbleLatCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			SBPL_4Dpt_t pose;
			pose.x = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.cellsize_m);
			pose.y = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.cellsize_m);
			pose.theta = angle;
			pose.t = DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution);
			envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
			envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
				tind, aind, envDBubbleLatCfg.ActionsV[tind][aind].endtheta, angle, 
				envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY,
				envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
			 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;
		envDBubbleLatCfg.ActionsV[tind][aind].endtheta = tind-1; //TODO:Mike ask Max if this should be modulo the number of angle directions like below
		if(envDBubbleLatCfg.ActionsV[tind][aind].endtheta < 0) envDBubbleLatCfg.ActionsV[tind][aind].endtheta += ENVDBUBBLELAT_THETADIRS;
		envDBubbleLatCfg.ActionsV[tind][aind].dX = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dY = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dT = (int)(envDBubbleLatCfg.timetoturn45degsinplace_secs);
		envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		SBPL_4Dpt_t pose;
		pose.x = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS);
		pose.t = DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution);
		envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
		envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envDBubbleLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS),
			envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY,
			envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
		 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));


		aind = 4;
		envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;
		envDBubbleLatCfg.ActionsV[tind][aind].endtheta = (tind + 1)%ENVDBUBBLELAT_THETADIRS; 
		envDBubbleLatCfg.ActionsV[tind][aind].dX = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dY = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dT = (int)(envDBubbleLatCfg.timetoturn45degsinplace_secs);
		envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS);
		pose.t = DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution);
		envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
		envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envDBubbleLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS),
			envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY,
			envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
		 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));

    //add one action for standing still
		aind = 5;
		envDBubbleLatCfg.ActionsV[tind][aind].starttheta = tind;
		envDBubbleLatCfg.ActionsV[tind][aind].endtheta = tind;
		envDBubbleLatCfg.ActionsV[tind][aind].dX = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dY = 0;
		envDBubbleLatCfg.ActionsV[tind][aind].dT = 1;
		envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*envDBubbleLatCfg.ActionsV[tind][aind].dT*envDBubbleLatCfg.timeResolution);
		//envDBubbleLatCfg.ActionsV[tind][aind].cost = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution));

		//compute intersecting cells
		pose.x = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dY, envDBubbleLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS);
		pose.t = DISCTIME2CONT(envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.timeResolution);
		envDBubbleLatCfg.ActionsV[tind][aind].intermptV.clear();
		envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envDBubbleLatCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envDBubbleLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envDBubbleLatCfg.ActionsV[tind][aind].endtheta, ENVDBUBBLELAT_THETADIRS),
			envDBubbleLatCfg.ActionsV[tind][aind].dX, envDBubbleLatCfg.ActionsV[tind][aind].dY,
			envDBubbleLatCfg.ActionsV[tind][aind].dT, envDBubbleLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = envDBubbleLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVDBUBBLELAT_THETADIRS;
		 envDBubbleLatCfg.PredActionsV[targettheta].push_back(&(envDBubbleLatCfg.ActionsV[tind][aind]));
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data\n");


}



void EnvDBubbleLattice::InitializeEnvConfig(vector<SBPL_xythetatimebubble_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of envDBubbleLatCfg if necessary

	//dXY dirs
	envDBubbleLatCfg.dXY[0][0] = -1;
	envDBubbleLatCfg.dXY[0][1] = -1;
	envDBubbleLatCfg.dXY[1][0] = -1;
	envDBubbleLatCfg.dXY[1][1] = 0;
	envDBubbleLatCfg.dXY[2][0] = -1;
	envDBubbleLatCfg.dXY[2][1] = 1;
	envDBubbleLatCfg.dXY[3][0] = 0;
	envDBubbleLatCfg.dXY[3][1] = -1;
	envDBubbleLatCfg.dXY[4][0] = 0;
	envDBubbleLatCfg.dXY[4][1] = 1;
	envDBubbleLatCfg.dXY[5][0] = 1;
	envDBubbleLatCfg.dXY[5][1] = -1;
	envDBubbleLatCfg.dXY[6][0] = 1;
	envDBubbleLatCfg.dXY[6][1] = 0;
	envDBubbleLatCfg.dXY[7][0] = 1;
	envDBubbleLatCfg.dXY[7][1] = 1;


	SBPL_4Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	temppose.t = 0.0;
	vector<SBPL_4Dcell_t> footprint;
	CalculateFootprintForPose(temppose, &footprint);
	SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (int)footprint.size());

#if DEBUG
	SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", footprint.size());
	for(int i = 0; i < (int) footprint.size(); i++)
	{
		SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y, 
			DISCXY2CONT(footprint.at(i).x, envDBubbleLatCfg.cellsize_m), 
			DISCXY2CONT(footprint.at(i).y, envDBubbleLatCfg.cellsize_m));
	}
#endif


	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);


}



bool EnvDBubbleLattice::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < envDBubbleLatCfg.EnvWidth_c && 
		Y >= 0 && Y < envDBubbleLatCfg.EnvHeight_c && 
		envDBubbleLatCfg.Grid2D[X][Y] < envDBubbleLatCfg.obsthresh);
}

bool EnvDBubbleLattice::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < envDBubbleLatCfg.EnvWidth_c && 
		Y >= 0 && Y < envDBubbleLatCfg.EnvHeight_c);
}

bool EnvDBubbleLattice::IsValidConfiguration(int X, int Y, int Theta)
{
	vector<SBPL_4Dcell_t> footprint;
	SBPL_4Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, envDBubbleLatCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, envDBubbleLatCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, ENVDBUBBLELAT_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (x < 0 || x >= envDBubbleLatCfg.EnvWidth_c ||
			y < 0 || Y >= envDBubbleLatCfg.EnvHeight_c ||		
			envDBubbleLatCfg.Grid2D[x][y] >= envDBubbleLatCfg.obsthresh)
		{
			return false;
		}
	}

	return true;
}


int EnvDBubbleLattice::GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceT, envDBubbleLatAction_t* action, vector<int>* bubbleCollisions)
{
	SBPL_4Dcell_t cell;
	SBPL_4Dcell_t interm4Dcell;
	int i;

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	
	if(!IsValidCell(SourceX, SourceY))
		return INFINITECOST;
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
		return INFINITECOST;

  //if(SourceX==1 && SourceY==1 && SourceTheta==0)
    //SBPL_PRINTF("1\n");

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	for(i = 0; i < (int)action->interm4DcellsV.size(); i++)
	{
		interm4Dcell = action->interm4DcellsV.at(i);
		interm4Dcell.x = interm4Dcell.x + SourceX;
		interm4Dcell.y = interm4Dcell.y + SourceY;
		interm4Dcell.t = interm4Dcell.t + SourceT;
		
		if(interm4Dcell.x < 0 || interm4Dcell.x >= envDBubbleLatCfg.EnvWidth_c ||
			interm4Dcell.y < 0 || interm4Dcell.y >= envDBubbleLatCfg.EnvHeight_c)
			return INFINITECOST;

		maxcellcost = __max(maxcellcost, envDBubbleLatCfg.Grid2D[interm4Dcell.x][interm4Dcell.y]);
		//check that the robot is NOT in the cell at which there is no valid orientation
		if(maxcellcost >= envDBubbleLatCfg.cost_inscribed_thresh)
			return INFINITECOST;
		if(getDynamicObstacleCost(interm4Dcell, bubbleCollisions) > envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh){
      return INFINITECOST;
    }
	}

  /*
  if(SourceX==1 && SourceY==1 && SourceTheta==0)
    SBPL_PRINTF("2\n");
  if(SourceX==1 && SourceY==1 && SourceTheta==0)
    SBPL_PRINTF("maxcell=%d possibly=%d\n",maxcellcost,envDBubbleLatCfg.cost_possibly_circumscribed_thresh);
    */

	//check collisions that for the particular footprint orientation along the action
	if(envDBubbleLatCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= envDBubbleLatCfg.cost_possibly_circumscribed_thresh)
	{
		checks++;

		for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
		{
			//get the cell in the map
			cell = action->intersectingcellsV.at(i);
			cell.x = cell.x + SourceX;
			cell.y = cell.y + SourceY;
      cell.t = cell.t + SourceT;
			
			//check validity
			if(!IsValidCell(cell.x, cell.y)) 
				return INFINITECOST;

      if(SourceX==1 && SourceY==1 && SourceTheta==0){
        /*
        for(int j = 0; j < (int)action->intersectingcellsV.size(); j++){
          SBPL_4Dcell_t cell2 = action->intersectingcellsV.at(j);
          SBPL_PRINTF("(%d,%d),",cell2.x,cell2.y);
        }
        SBPL_PRINTF("\n");
        */
      }

			//if(envDBubbleLatCfg.Grid2D[cell.x][cell.y] > currentmaxcost) //cost computation changed: cost = max(cost of centers of the robot along action)
			//	currentmaxcost = envDBubbleLatCfg.Grid2D[cell.x][cell.y];	//intersecting cells are only used for collision checking
		}
	}

  //if(SourceX==1 && SourceY==1 && SourceTheta==0)
    //SBPL_PRINTF("3\n");

	//to ensure consistency of h2D:
	maxcellcost = __max(maxcellcost, envDBubbleLatCfg.Grid2D[SourceX][SourceY]);
	int currentmaxcost = (int)__max(maxcellcost, envDBubbleLatCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

	return action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}

unsigned char EnvDBubbleLattice::getDynamicObstacleCost(SBPL_4Dcell_t cell, vector<int>* bubbleCollisions){
  //TODO:This function is slightly suboptimal!!! If there is a collision with a dynamic obstacle in this cell then I activate all bubbles of that 
  //dynamic obstacle that go through this cell even if they are not part of the collision!  To fix this I need to change the BubbleCellObs structs
  //to have a startt and endt vector with one for each bubble ID...not sure if it is worth it since it will slow down the collision checking and those
  //bubbles are likely to be activated shortly after anyway...

  if(cell.t > bubblemap[cell.x][cell.y].lastObsT || cell.t < bubblemap[cell.x][cell.y].firstObsT){
    //SBPL_PRINTF("t=%d first=%d last=%d\n",cell.t,bubblemap[cell.x][cell.y].firstObsT,bubblemap[cell.x][cell.y].lastObsT);
    return 0;
  }
  vector<envDBubbleLat_BubbleCellObs_t> dynObs = bubblemap[cell.x][cell.y].dynObs;
  //no cost if we are not in any dynamic obstacle bubbles
  if(dynObs.size() == 0)
    return 0;
  
  //TODO:Mike this currently does not take into account the trajectory probability
  bool collision = false;
  for(unsigned int i=0; i < dynObs.size(); i++){
    if(cell.t <= dynObs[i].collision_endt && cell.t >= dynObs[i].collision_startt){
      if(bubbleCollisions){
        for(unsigned int m=0; m<dynObs[i].ID.size(); m++){
          int id = dynObs[i].ID[m];
          if(!bubble4Dactive[id]){
            bubbleCollisions->push_back(id);
            bubble4Dactive[id] = true;
          }
        }
      }
      else
        return 255;
      collision = true;
    }
  }
  
  if(collision)
    return 255;

  return 0;
}
/*
unsigned char EnvDBubbleLattice::getDynamicObstacleCost(SBPL_4Dcell_t cell, vector<int>* bubbleCollisions){
  //TODO:This function is slightly suboptimal!!! If there is a collision with a dynamic obstacle in this cell then I activate all bubbles of that 
  //dynamic obstacle that go through this cell even if they are not part of the collision!  To fix this I need to change the BubbleCellObs structs
  //to have a startt and endt vector with one for each bubble ID...may as well also make one more vector for inner or outer bubble so we can do 
  //collision checks in O(1)...instead of going through the whole trajectory like in the function...
  //if(cell.t > bubblemap[cell.x][cell.y].lastObsT || cell.t < bubblemap[cell.x][cell.y].firstObsT){
    //SBPL_PRINTF("t=%d first=%d last=%d\n",cell.t,bubblemap[cell.x][cell.y].firstObsT,bubblemap[cell.x][cell.y].lastObsT);
    //return 0;
  //}
  vector<envDBubbleLat_BubbleCellObs_t> dynObs = bubblemap[cell.x][cell.y].dynObs;
  //no cost if we are not in any dynamic obstacle bubbles
  if(dynObs.size() == 0)
    return 0;

  //TODO:Mike this currently does not take into account the trajectory probability
  bool collision = false;
  for(int i=0; i < dynObs.size(); i++){
    //if we are here before or after this dynamic obstacle has passed through then we don't need to collision check against it
    //if(cell.t > dynObs[i].endt || cell.t < dynObs[i].startt)
      //continue;
    SBPL_DynamicObstacle_t obs = *(dynObs[i].obs);
    for(int j=0; j < obs.trajectories.size(); j++){
      SBPL_Trajectory_t traj = obs.trajectories.at(j);
      int k = 0;
      for(; k < traj.points.size(); k++){
        if(traj.points.at(k).t > cell.t)
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
        double dist = dynObsPtSqrDist(cell, p);
        if(sqrt(dist) <= sqrt(obs.radiusSquared) + 3*p.std_dev){
          if(bubbleCollisions){
            for(int m=0; m<dynObs[i].ID.size(); m++){
              int id = dynObs[i].ID[m];
              if(!bubble4Dactive[id]){
                bubbleCollisions->push_back(id);
                bubble4Dactive[id] = true;
              }
            }
          }
          collision = true;
        }
      }
      else{
        //p1 and p2 are the points before and after cell (in terms of time) respectively
        SBPL_Traj_Pt_t p1, p2;
        p1 = traj.points.at(k-1);
        double dist = dynObsPtSqrDist(cell, p1);
        if(sqrt(dist) <= sqrt(obs.radiusSquared) + 3*p1.std_dev){
          if(bubbleCollisions){
            for(int m=0; m<dynObs[i].ID.size(); m++){
              int id = dynObs[i].ID[m];
              if(!bubble4Dactive[id]){
                bubbleCollisions->push_back(id);
                bubble4Dactive[id] = true;
              }
            }
          }
          collision = true;
        }
        p2 = traj.points.at(k);
        dist = dynObsPtSqrDist(cell, p2);
        if(sqrt(dist) <= sqrt(obs.radiusSquared) + 3*p2.std_dev){
          if(bubbleCollisions){
            for(int m=0; m<dynObs[i].ID.size(); m++){
              int id = dynObs[i].ID[m];
              if(!bubble4Dactive[id]){
                bubbleCollisions->push_back(id);
                bubble4Dactive[id] = true;
              }
            }
          }
          collision = true;
        }
      }
      
    }
  }

  if(collision)
    return 255;

  //no collision
  return 0;
}

double EnvDBubbleLattice::dynObsPtSqrDist(SBPL_4Dcell_t cell, SBPL_Traj_Pt_t p){
  double x = DISCXY2CONT(cell.x, envDBubbleLatCfg.cellsize_m);
  double y = DISCXY2CONT(cell.y, envDBubbleLatCfg.cellsize_m);
  return (x-p.x)*(x-p.x)+(y-p.y)*(y-p.y);
}
*/

double EnvDBubbleLattice::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return envDBubbleLatCfg.cellsize_m*sqrt((double)sqdist);

}


//adds points to it (does not clear it beforehand)
void EnvDBubbleLattice::CalculateFootprintForPose(SBPL_4Dpt_t pose, vector<SBPL_4Dcell_t>* footprint)
{  
	int pind;

#if DEBUG
//  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
//	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(envDBubbleLatCfg.FootprintPolygon.size() <= 1){
    SBPL_4Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, envDBubbleLatCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, envDBubbleLatCfg.cellsize_m);

	for(pind = 0; pind < (int)footprint->size(); pind++)
	{
		if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
			break;
	}
	if(pind == (int)footprint->size()) footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt = {0,0};
  for(find = 0; find < envDBubbleLatCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = envDBubbleLatCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
#if DEBUG
//    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

#if DEBUG
//  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, envDBubbleLatCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, envDBubbleLatCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=envDBubbleLatCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=envDBubbleLatCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, envDBubbleLatCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, envDBubbleLatCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
//		SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){
		//convert to a grid point

#if DEBUG
//			SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

			SBPL_4Dcell_t cell;
			cell.x = discrete_x;
			cell.y = discrete_y;

			//insert point if not there already
			int pind = 0;
			for(pind = 0; pind < (int)footprint->size(); pind++)
			{
				if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
					break;
			}
			if(pind == (int)footprint->size()) footprint->push_back(cell);

			prev_inside = 1;

#if DEBUG
//			SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }
}


void EnvDBubbleLattice::RemoveSourceFootprint(SBPL_4Dpt_t sourcepose, vector<SBPL_4Dcell_t>* footprint)
{  
	vector<SBPL_4Dcell_t> sourcefootprint;

	//compute source footprint
	CalculateFootprintForPose(sourcepose, &sourcefootprint);

	//now remove the source cells from the footprint
	for(int sind = 0; sind < (int)sourcefootprint.size(); sind++)
	{
		for(int find = 0; find < (int)footprint->size(); find++)
		{
			if(sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y == footprint->at(find).y)
			{
				footprint->erase(footprint->begin() + find);
				break;
			}
		}//over footprint
	}//over source



}


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvDBubbleLattice::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	SBPL_PRINTF("Precomputing heuristics...\n");
	
	//allocated 2D grid searches
	grid2Dsearchfromstart = new SBPL2DGridSearch(envDBubbleLatCfg.EnvWidth_c, envDBubbleLatCfg.EnvHeight_c, (float)envDBubbleLatCfg.cellsize_m);
	grid2Dsearchfromgoal = new SBPL2DGridSearch(envDBubbleLatCfg.EnvWidth_c, envDBubbleLatCfg.EnvHeight_c, (float)envDBubbleLatCfg.cellsize_m);

	SBPL_PRINTF("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvDBubbleLattice::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/ENVDBUBBLELAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, ENVDBUBBLELAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, ENVDBUBBLELAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, ENVDBUBBLELAT_THETADIRS);

		SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);

        if(nTheta != nnewTheta)
        {
            SBPL_ERROR("ERROR: invalid quantization\n");                     
            return false;
        }
    }

  return true;
}



//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------
bool EnvDBubbleLattice::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sDynObsFile){
  //load the environment and the motion primitives
  InitializeEnv(sEnvFile, perimeterptsV, sMotPrimFile);

  //load the dynamic obstacles
  FILE* fDynObs = fopen(sDynObsFile, "r");
  if(fDynObs == NULL){
    SBPL_PRINTF("Error: unable to open %s\n", sDynObsFile);
    throw new SBPL_Exception();
  }
  ReadDynamicObstacles(fDynObs);
  fclose(fDynObs);
  UpdateBubbleMap();

  return true;
}

bool EnvDBubbleLattice::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile)
{
  //find the radius of the robot (smallest circle that contains the robot) squared
  //this will be used to pad dynamic obstacles for faster collision checking
  double max_sqr_radius = 0;
  for(unsigned int i=0; i<perimeterptsV.size(); i++){
    double x = perimeterptsV.at(i).x;
    double y = perimeterptsV.at(i).y;
    double d = x*x + y*y;
    if(d > max_sqr_radius)
      max_sqr_radius = d;
  }
  envDBubbleLatCfg.robotRadius = sqrt(max_sqr_radius);
  SBPL_PRINTF("robotRadius=%f\n", envDBubbleLatCfg.robotRadius);

	envDBubbleLatCfg.FootprintPolygon = perimeterptsV;

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	ReadConfiguration(fCfg);
  fclose(fCfg);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}
		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
    fclose(fMotPrim);
		InitGeneral(&envDBubbleLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);


  temporal_padding = DEFAULT_TEMPORAL_PADDING;
  InitializeBubbleMap();
	return true;
}


bool EnvDBubbleLattice::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	ReadConfiguration(fCfg);

	InitGeneral(NULL);


	return true;
}

bool EnvDBubbleLattice::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh, unsigned char dynobsthresh,  const char* sMotPrimFile,
          vector<SBPL_DynamicObstacle_t> & dynObs)
{

	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f timeResolution=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
		width, height, startx, starty, starttheta, startTime, goalx, goaly, goaltheta, cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	SBPL_PRINTF("perimeter has size=%d\n", (int)perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	envDBubbleLatCfg.obsthresh = obsthresh;
	envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

  //Set up the dynamic obstacles...
  double max_sqr_radius = 0;
  for(unsigned int i=0; i<perimeterptsV.size(); i++){
    double x = perimeterptsV.at(i).x;
    double y = perimeterptsV.at(i).y;
    double d = x*x + y*y;
    if(d > max_sqr_radius)
      max_sqr_radius = d;
  }
  envDBubbleLatCfg.robotRadius = sqrt(max_sqr_radius);
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + envDBubbleLatCfg.robotRadius;
    for(unsigned int j=0; j<dynObs[i].trajectories.size(); j++){
      SBPL_Trajectory_t traj;
      traj.prob = dynObs[i].trajectories[j].prob;
      traj.existsAfter = dynObs[i].trajectories[j].existsAfter;
      for(unsigned int k=0; k<dynObs[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p;
        p.x = dynObs[i].trajectories[j].points[k].x;
        p.y = dynObs[i].trajectories[j].points[k].y;
        p.t = dynObs[i].trajectories[j].points[k].t;
        p.std_dev = dynObs[i].trajectories[j].points[k].std_dev;
        traj.points.push_back(p);
      }
      obs.trajectories.push_back(traj);
    }
    dynamicObstacles.push_back(obs);
  }

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, ENVDBUBBLELAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, ENVDBUBBLELAT_THETADIRS),
					cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
	}

	if(envDBubbleLatCfg.mprimV.size() != 0)
	{
		InitGeneral(&envDBubbleLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);

  temporal_padding = DEFAULT_TEMPORAL_PADDING;
  InitializeBubbleMap();
	return true;
}


bool EnvDBubbleLattice::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh, unsigned char dynobsthresh,  const char* sMotPrimFile)
{

	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f timeResolution=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
		width, height, startx, starty, starttheta, startTime, goalx, goaly, goaltheta, cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	SBPL_PRINTF("perimeter has size=%d\n", (int)perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	envDBubbleLatCfg.obsthresh = obsthresh;
	envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, ENVDBUBBLELAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, ENVDBUBBLELAT_THETADIRS),
					cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
	}

	if(envDBubbleLatCfg.mprimV.size() != 0)
	{
		InitGeneral(&envDBubbleLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}


bool EnvDBubbleLattice::InitGeneral(vector<SBPL_xythetatimebubble_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);

  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvDBubbleLattice::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = envDBubbleLat.goalstateid;
	MDPCfg->startstateid = envDBubbleLat.startstateid;

	return true;
}


void EnvDBubbleLattice::PrintHeuristicValues()
{
	FILE* fHeur = fopen("heur.txt", "w");
	SBPL2DGridSearch* grid2Dsearch = NULL;
	
	for(int i = 0; i < 2; i++)
	{
		if(i == 0 && grid2Dsearchfromstart != NULL)
		{
			grid2Dsearch = grid2Dsearchfromstart;
			SBPL_FPRINTF(fHeur, "start heuristics:\n");
		}
		else if(i == 1 && grid2Dsearchfromgoal != NULL)
		{
			grid2Dsearch = grid2Dsearchfromgoal;
			SBPL_FPRINTF(fHeur, "goal heuristics:\n");
		}
		else
			continue;

		for (int y = 0; y < envDBubbleLatCfg.EnvHeight_c; y++) {
			for (int x = 0; x < envDBubbleLatCfg.EnvWidth_c; x++) {
				if(grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
					SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
			else
				SBPL_FPRINTF(fHeur, "XXXXX ");
			}
			SBPL_FPRINTF(fHeur, "\n");
		}
	}
	fclose(fHeur);
}




void EnvDBubbleLattice::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	SBPL_ERROR("ERROR in envDBubbleLat... function: SetAllPreds is undefined\n");
	throw new SBPL_Exception();
}


void EnvDBubbleLattice::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	GetSuccs(SourceStateID, SuccIDV, CostV, NULL, NULL, NULL);
}


void EnvDBubbleLattice::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* stateBubbles, vector<int>* bubbleCollisions)
{
	GetSuccs(SourceStateID, SuccIDV, CostV, stateBubbles, bubbleCollisions, NULL);
}



const envDBubbleLatConfig_t* EnvDBubbleLattice::GetEnvNavConfig() {
  return &envDBubbleLatCfg;
}



bool EnvDBubbleLattice::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,envDBubbleLatCfg.Grid2D[x][y], newcost);
#endif

    envDBubbleLatCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

    return true;
}


void EnvDBubbleLattice::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out envDBubbleLat. configuration
	
	SBPL_ERROR("ERROR in envDBubbleLat... function: PrintEnv_Config is undefined\n");
	throw new SBPL_Exception();

}

void EnvDBubbleLattice::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}



bool EnvDBubbleLattice::IsObstacle(int x, int y)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,envDBubbleLatCfg.Grid2D[x][y]);
#endif


	return (envDBubbleLatCfg.Grid2D[x][y] >= envDBubbleLatCfg.obsthresh); 

}

int EnvDBubbleLattice::getStartID(){
  return envDBubbleLat.startstateid;
}

int EnvDBubbleLattice::getGoalID(){
  return envDBubbleLat.goalstateid;
}

void EnvDBubbleLattice::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* startTime, double* goalx, double* goaly, double* goaltheta,
									  	double* cellsize_m, double* timeResolution, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, unsigned char* dyn_obs_thresh,
										vector<SBPL_xythetatimebubble_mprimitive>* mprimitiveV)
{
	*size_x = envDBubbleLatCfg.EnvWidth_c;
	*size_y = envDBubbleLatCfg.EnvHeight_c;

	*startx = DISCXY2CONT(envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.cellsize_m);
	*starty = DISCXY2CONT(envDBubbleLatCfg.StartY_c, envDBubbleLatCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(envDBubbleLatCfg.StartTheta, ENVDBUBBLELAT_THETADIRS);
	*startTime = DISCTIME2CONT(envDBubbleLatCfg.StartTime, envDBubbleLatCfg.timeResolution);
	*goalx = DISCXY2CONT(envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.cellsize_m);
	*goaly = DISCXY2CONT(envDBubbleLatCfg.EndY_c, envDBubbleLatCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(envDBubbleLatCfg.EndTheta, ENVDBUBBLELAT_THETADIRS);

	*cellsize_m = envDBubbleLatCfg.cellsize_m;
	*timeResolution = envDBubbleLatCfg.timeResolution;
	*nominalvel_mpersecs = envDBubbleLatCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = envDBubbleLatCfg.timetoturn45degsinplace_secs;

	*dyn_obs_thresh = envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh;
	*obsthresh = envDBubbleLatCfg.obsthresh;

	*mprimitiveV = envDBubbleLatCfg.mprimV;
}

bool EnvDBubbleLattice::PoseContToDisc(double px, double py, double pth, double pt,
					 int &ix, int &iy, int &ith, int &it) const
{
  ix = CONTXY2DISC(px, envDBubbleLatCfg.cellsize_m);
  iy = CONTXY2DISC(py, envDBubbleLatCfg.cellsize_m);
  ith = ContTheta2Disc(pth, ENVDBUBBLELAT_THETADIRS); // ContTheta2Disc() normalizes the angle
  it = CONTTIME2DISC(pt, envDBubbleLatCfg.timeResolution);
  return (pth >= -2*PI_CONST) && (pth <= 2*PI_CONST)
    && (ix >= 0) && (ix < envDBubbleLatCfg.EnvWidth_c)
    && (iy >= 0) && (iy < envDBubbleLatCfg.EnvHeight_c);
}

bool EnvDBubbleLattice::PoseDiscToCont(int ix, int iy, int ith, int it,
					 double &px, double &py, double &pth, double &pt) const
{
  px = DISCXY2CONT(ix, envDBubbleLatCfg.cellsize_m);
  py = DISCXY2CONT(iy, envDBubbleLatCfg.cellsize_m);
  pth = normalizeAngle(DiscTheta2Cont(ith, ENVDBUBBLELAT_THETADIRS));
  pt = DISCTIME2CONT(it, envDBubbleLatCfg.timeResolution);
  return (ith >= 0) && (ith < ENVDBUBBLELAT_THETADIRS)
    && (ix >= 0) && (ix < envDBubbleLatCfg.EnvWidth_c)
    && (iy >= 0) && (iy < envDBubbleLatCfg.EnvHeight_c);
}

unsigned char EnvDBubbleLattice::GetMapCost(int x, int y)
{
	return envDBubbleLatCfg.Grid2D[x][y];
}



bool EnvDBubbleLattice::SetEnvParameter(const char* parameter, int value)
{

	if(envDBubbleLat.bInitialized == true)
	{
		SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
		return false;
	}

	SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envDBubbleLatCfg.cost_inscribed_thresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envDBubbleLatCfg.cost_possibly_circumscribed_thresh = value;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envDBubbleLatCfg.obsthresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh = (unsigned char)value;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		return false;
	}

	return true;
}

int EnvDBubbleLattice::GetEnvParameter(const char* parameter)
{

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		return (int) envDBubbleLatCfg.cost_inscribed_thresh;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		return (int) envDBubbleLatCfg.cost_possibly_circumscribed_thresh;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		return (int) envDBubbleLatCfg.obsthresh;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		return (int) envDBubbleLatCfg.dynamic_obstacle_collision_cost_thresh;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		throw new SBPL_Exception();
	}

}

//------------------------------------------------------------------------------



//-----------------XYTHETA Enivornment (child) class---------------------------
void EnvDBubbleLat::GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t) const {
  envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  t = HashEntry->T;
}

int EnvDBubbleLat::GetStateFromCoord(int x, int y, int theta, int t){

  envDBubbleLatHashEntry_t* OutHashEntry;
  if((OutHashEntry = GetHashEntry(x, y, theta, t, isInBubble(x,y,t))) == NULL){
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y, theta, t, isInBubble(x,y,t));
  }
  return OutHashEntry->stateID;
}
/*
void EnvDBubbleLat::PostProcess(vector<int>* states, vector<int>* cost){
  int posTurn = 0;
  int negTurn = 1;
  int wait = 2;
  int other = 3;
  int lastAction = other;

  int posTurnCount = 0;
  int posTurnStart = -1;
  int negTurnCount = 0;
  int negTurnStart = -1;

  vector<int> new_states;
  vector<int> new_cost;
  
  for(int i=0; i<states.size()-1; i++){
    int sourceID = states[i];
    int targetID = states[i+1];
    envDBubbleLatHashEntry_t* source = StateID2CoordTable[sourceID];
    envDBubbleLatHashEntry_t* target = StateID2CoordTable[targetID];
    
    if(source->x != target->x || source->y != target->y){
      int numWaits = min(posTurnCount, negTurnCount)*waitsPerTurn;
      insertWaits(&new_states, &new_cost, numWaits);
      
      if(posTurnCount > 0 && negTurnCount > 0){
        //replace a left with waits and a right with waits
        
        posTurnCount--;
        negTurnCount--;
      }

      new_states.push_back(states[i]);
      new_cost.push_back(cost[i]);
      
      posTurnCount = 0;
      posTurnStart = -1;
      negTurnCount = 0;
      negTurnStart = -1;
      lastAction = other;
    }
    else if(source->theta == target->theta){
      //wait
      new_states.push_back(states[i]);
      new_cost.push_back(cost[i]);
      //lastAction = wait;
    }
    else if((target->theta - source->theta)%NUMTHETADIRS < (source->theta - target->theta)%NUMTHETADIRS){
      //turn one way
      if(posTurnStart == -1)
        posTurnStart = i;
      posTurnCount++;
      lastAction = posTurn;
    }
    else{
      //turn other way
      if(negTurnStart == -1)
        negTurnStart = i;
      negTurnCount++;
      lastAction = negTurn;
    }
  }

  *states = new_states;
  *cost = new_cost;
}
*/
void EnvDBubbleLat::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath)
{
	vector<envDBubbleLatAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
	int targetx_c, targety_c, targettheta_c;
	int sourcex_c, sourcey_c, sourcetheta_c;
  int targett_c, sourcet_c;

	//SBPL_PRINTF("checks=%ld\n", checks);

	xythetaPath->clear();

#if DEBUG
	SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

	for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
	{
		int sourceID = stateIDPath->at(pind);
    //envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[sourceID];
    //SBPL_PRINTF("stateid=%d x=%d, y=%d, theta=%d, t=%d bubble=%d\n",sourceID,HashEntry->X,HashEntry->Y,HashEntry->Theta,HashEntry->T,HashEntry->inBubble);
		int targetID = stateIDPath->at(pind+1);

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
#endif


		//get successors and pick the target via the cheapest action
		SuccIDV.clear();
		CostV.clear();
		actionV.clear();
		GetSuccs(sourceID, &SuccIDV, &CostV, NULL, NULL, &actionV);
		
		int bestcost = INFINITECOST;
		int bestsind = -1;

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c);
		SBPL_FPRINTF(fDeb, "looking for %d %d %d %d -> %d %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c, sourcet_c
					targetx_c, targety_c, targettheta_c, targett_c, SuccIDV.size()); 

#endif


		for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
		{

#if DEBUG
		int x_c, y_c, theta_c, t_c;
		GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, t_c);
		SBPL_FPRINTF(fDeb, "succ: %d %d %d %d\n", x_c, y_c, theta_c, t_c); 
#endif

			if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
			{
				bestcost = CostV[sind];
				bestsind = sind;
			}
		}
		if(bestsind == -1)
		{
			SBPL_ERROR("ERROR: successor not found for transition:\n");
			GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
			GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c);
			SBPL_PRINTF("ID=%d %d %d %d %d -> ID=%d %d %d %d %d\n", sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c,
					targetID, targetx_c, targety_c, targettheta_c, targett_c); 
			throw new SBPL_Exception();
		}

		//now push in the actual path
		int sourcex_c, sourcey_c, sourcetheta_c;
		int sourcet_c;
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
		double sourcex, sourcey, sourcet;
		sourcex = DISCXY2CONT(sourcex_c, envDBubbleLatCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, envDBubbleLatCfg.cellsize_m);
		sourcet = DISCTIME2CONT(sourcet_c, envDBubbleLatCfg.timeResolution);
		//TODO - when there are no motion primitives we should still print source state
		for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
		{
			//translate appropriately
			SBPL_4Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
			intermpt.x += sourcex;
			intermpt.y += sourcey;
			intermpt.t += sourcet;

#if DEBUG
			int nx = CONTXY2DISC(intermpt.x, envDBubbleLatCfg.cellsize_m);
			int ny = CONTXY2DISC(intermpt.y, envDBubbleLatCfg.cellsize_m);
			int nt = CONTTIME2DISC(intermpt.t, envDBubbleLatCfg.timeResolution);
			SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f %.3f (%d %d %d cost=%d) ", 
				intermpt.x, intermpt.y, intermpt.theta, intermpt.t
				nx, ny, 
				ContTheta2Disc(intermpt.theta, ENVDBUBBLELAT_THETADIRS), nt, envDBubbleLatCfg.Grid2D[nx][ny]);
			if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
			else SBPL_FPRINTF(fDeb, "\n");
#endif

			//store
			xythetaPath->push_back(intermpt);
		}
	}
}


//returns the stateid if success, and -1 otherwise
int EnvDBubbleLat::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, envDBubbleLatCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, envDBubbleLatCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, ENVDBUBBLELAT_THETADIRS);

	SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
		SBPL_PRINTF("WARNING: goal configuration is invalid\n");
	}

    envDBubbleLatHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, INFINITECOST, false)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, INFINITECOST, false);
    }

	//need to recompute start heuristics?
	//if(envDBubbleLat.goalstateid != OutHashEntry->stateID)
  int oldGoalX, oldGoalY, oldGoalTheta;
  int oldGoalT;
  GetCoordFromState(envDBubbleLat.goalstateid, oldGoalX, oldGoalY, oldGoalTheta, oldGoalT);
  if(oldGoalX != x || oldGoalY != y || oldGoalTheta != theta)
	{
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
		bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
	}



    envDBubbleLat.goalstateid = OutHashEntry->stateID;

	envDBubbleLatCfg.EndX_c = x;
	envDBubbleLatCfg.EndY_c = y;
	envDBubbleLatCfg.EndTheta = theta;


    return envDBubbleLat.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvDBubbleLat::SetStart(double x_m, double y_m, double theta_rad, double startTime){

	int x = CONTXY2DISC(x_m, envDBubbleLatCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, envDBubbleLatCfg.cellsize_m); 
	int theta = ContTheta2Disc(theta_rad, ENVDBUBBLELAT_THETADIRS);
  int t = CONTTIME2DISC(startTime, envDBubbleLatCfg.timeResolution);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
		return -1;
	}

	SBPL_PRINTF("env: setting start to %.3f %.3f %.3f %.3f (%d %d %d %d)\n", x_m, y_m, theta_rad, startTime, x, y, theta, t);

    if(!IsValidConfiguration(x,y,theta))
	{
		SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x,y,theta);
	}

    envDBubbleLatHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, t, false)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, t, false);
    }

	//need to recompute start heuristics?
	//if(envDBubbleLat.startstateid != OutHashEntry->stateID)
  int oldStartX, oldStartY, oldStartTheta;
  int oldStartT;
  GetCoordFromState(envDBubbleLat.startstateid, oldStartX, oldStartY, oldStartTheta, oldStartT);
  if(oldStartX != x || oldStartY != y || oldStartTheta != theta)
	{
		bNeedtoRecomputeStartHeuristics = true;
		bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
	}

	//set start
    envDBubbleLat.startstateid = OutHashEntry->stateID;
	envDBubbleLatCfg.StartX_c = x;
	envDBubbleLatCfg.StartY_c = y;
	envDBubbleLatCfg.StartTheta = theta;
	envDBubbleLatCfg.StartTime = t;

    return envDBubbleLat.startstateid;    

}

bool EnvDBubbleLat::setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states){
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + envDBubbleLatCfg.robotRadius;
    for(unsigned int j=0; j<dynObs[i].trajectories.size(); j++){
      SBPL_Trajectory_t traj;
      traj.prob = dynObs[i].trajectories[j].prob;
      traj.existsAfter = dynObs[i].trajectories[j].existsAfter;
      for(unsigned int k=0; k<dynObs[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p;
        p.x = dynObs[i].trajectories[j].points[k].x;
        p.y = dynObs[i].trajectories[j].points[k].y;
        p.t = dynObs[i].trajectories[j].points[k].t;
        p.std_dev = dynObs[i].trajectories[j].points[k].std_dev;
        traj.points.push_back(p);
        //SBPL_PRINTF("%f %f %f %f\n",p.x,p.y,DISCTIME2CONT(p.t, envDBubbleLatCfg.timeResolution),p.std_dev);
      }
      obs.trajectories.push_back(traj);
    }
    dynamicObstacles.push_back(obs);
  }
  SBPL_PRINTF("we have %d dynamic obstacles\n", (int)dynamicObstacles.size());
  //SBPL_Traj_Pt_t p1 = dynamicObstacles[0].trajectories[0].points[0];
  //SBPL_Traj_Pt_t p2 = dynamicObstacles[0].trajectories[0].points[dynamicObstacles[0].trajectories[0].points.size()-1];
  //SBPL_PRINTF("start=(%f, %f, %d) end=(%f, %f, %d)\n", p1.x, p1.y, p1.t, p2.x, p2.y, p2.t);

  UpdateBubbleMap();
  return true;
}

void EnvDBubbleLat::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envDBubbleLat... function: stateID illegal (2)\n");
		throw new SBPL_Exception();
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	envDBubbleLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envDBubbleLat.goalstateid];

	if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta && bVerbose)
	{
		SBPL_FPRINTF(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d T=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T);
    else
    	SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, envDBubbleLatCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,envDBubbleLatCfg.cellsize_m), 
		DiscTheta2Cont(HashEntry->Theta, ENVDBUBBLELAT_THETADIRS), DISCTIME2CONT(HashEntry->T, envDBubbleLatCfg.timeResolution));

}


envDBubbleLatHashEntry_t* EnvDBubbleLat::GetHashEntry(int X, int Y, int Theta, int T, bool inBubble)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif
  
  int binid;
  if(!inBubble)
    binid = GETHASHBIN(X, Y, Theta, -1);
  else
    binid = GETHASHBIN(X, Y, Theta, T);
	
#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 500)
	{
		SBPL_PRINTF("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
  if(inBubble){
    for(int ind = 0; ind < (int)Coord2StateIDHashTable[binid].size(); ind++)
    {
      if( Coord2StateIDHashTable[binid][ind]->X == X 
        && Coord2StateIDHashTable[binid][ind]->Y == Y
        && Coord2StateIDHashTable[binid][ind]->Theta == Theta
        && Coord2StateIDHashTable[binid][ind]->T == T
        && Coord2StateIDHashTable[binid][ind]->inBubble == inBubble)
      {
#if TIME_DEBUG
        time_gethash += clock()-currenttime;
#endif
        return Coord2StateIDHashTable[binid][ind];
      }
    }
  }
  else{//out of bubble so time doesn't matter
    for(int ind = 0; ind < (int)Coord2StateIDHashTable[binid].size(); ind++)
    {
      if( Coord2StateIDHashTable[binid][ind]->X == X 
        && Coord2StateIDHashTable[binid][ind]->Y == Y
        && Coord2StateIDHashTable[binid][ind]->Theta == Theta
        && Coord2StateIDHashTable[binid][ind]->inBubble == inBubble)
      {
#if TIME_DEBUG
        time_gethash += clock()-currenttime;
#endif
        return Coord2StateIDHashTable[binid][ind];
      }
    }

  }

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


envDBubbleLatHashEntry_t* EnvDBubbleLat::CreateNewHashEntry(int X, int Y, int Theta, int T, bool inBubble) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	envDBubbleLatHashEntry_t* HashEntry = new envDBubbleLatHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
  HashEntry->inBubble = inBubble;
  HashEntry->T = T;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
  if(HashEntry->inBubble)
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T); 
  else
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, -1); 
    

	//insert the entry into the bin
  Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND];
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
		throw new SBPL_Exception();	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}


bool EnvDBubbleLattice::isInBubble(int x, int y, int t){
  //SBPL_PRINTF("bubblemap.size=%d x=%d\n", bubblemap.size(), x);
  //SBPL_PRINTF("bubblemap[x].size=%d y=%d\n", bubblemap[x].size(), y);
  return x >= 0 && x < envDBubbleLatCfg.EnvWidth_c &&
         y >= 0 && y < envDBubbleLatCfg.EnvHeight_c &&
         bubblemap[x][y].dynObs.size() > 0 && bubblemap[x][y].lastObsT >= t;
}

void EnvDBubbleLat::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* stateBubbles /*=NULL*/, vector<int>* bubbleCollisions /*=NULL*/, vector<envDBubbleLatAction_t*>* actionV /*=NULL*/)
{
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(envDBubbleLatCfg.actionwidth); 
  CostV->reserve(envDBubbleLatCfg.actionwidth);
	if(actionV != NULL)
	{
		actionV->clear();
		actionV->reserve(envDBubbleLatCfg.actionwidth);
	}

	//get X, Y for the state
	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
	envDBubbleLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envDBubbleLat.goalstateid];
	
  
  //if(SourceStateID==9258)
  if(actionV != NULL)
    SBPL_PRINTF("stateid=%d x=%d, y=%d, theta=%d, t=%d bubble=%d\n",SourceStateID,HashEntry->X,HashEntry->Y,HashEntry->Theta,HashEntry->T,HashEntry->inBubble);

  if(SourceStateID == envDBubbleLat.goalstateid){
    return;
  }

	//iterate through actions
	for (aind = 0; aind < envDBubbleLatCfg.actionwidth; aind++)
	{
    //The "stand still" motion primitive is hard-coded to be the last motion in the list
    //we can only use the "stand still" motion primitive if we started in a bubble or any one of our actions puts us in a bubble
    if(aind == envDBubbleLatCfg.actionwidth-1 && !isInBubble(HashEntry->X, HashEntry->Y, HashEntry->T) /*!HashEntry->inBubble*/)
      break;
    else if(aind == envDBubbleLatCfg.actionwidth-1 && isInBubble(HashEntry->X, HashEntry->Y, HashEntry->T) /*HashEntry->inBubble*/){
      /*if(stateBubbles){
        bool active = false;
        envDBubbleLat_BubbleCell_t cell = bubblemap[HashEntry->X][HashEntry->Y];
        for(int i=0; i<cell.dynObs.size(); i++)
          for(int j=0; j<cell.dynObs[i].ID.size(); j++)
            active = active || bubble4Dactive[cell.dynObs[i].ID[j]];
        if(!active){
          for(int i=0; i<cell.dynObs.size(); i++)
            for(int j=0; j<cell.dynObs[i].ID.size(); j++){
              stateBubbles->push_back(cell.dynObs[i].ID[j]);
            }
        }
      }*/
    }

    //compute new pose from this aind action (successor pose)
    envDBubbleLatAction_t* nav4daction = &envDBubbleLatCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, ENVDBUBBLELAT_THETADIRS);	
    int newT = HashEntry->T + nav4daction->dT;

    //skip the invalid cells
    if(!IsValidCell(newX, newY)) 
      continue;

    bool newInBubble = isInBubble(newX, newY, newT);
    //if(newInBubble)
      //SBPL_PRINTF("what???\n");
    /*
    //determine if this action is outside the bubble
    bool newInBubble = true;
    if(aind != envDBubbleLatCfg.actionwidth-1){
      //if we are outside all bubbles then make the the newInBubble flag false so we can hash it without time
      bool active = false;
      envDBubbleLat_BubbleCell_t cell = bubblemap[newX][newY];
      for(int i=0; i<cell.dynObs.size(); i++){
        if(newT <= cell.dynObs[i].endt){
          for(int j=0; j<cell.dynObs[i].ID.size(); j++)
            active = active || bubble4Dactive[cell.dynObs[i].ID[j]];
        }
      }
      if(!active)
        newInBubble = false;
    }
    */

		//get cost
		int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav4daction, bubbleCollisions);
    if(cost >= INFINITECOST)
      continue;

    envDBubbleLatHashEntry_t* OutHashEntry;
    //TODO:Mike let this have some close enough threshold
    if(newX == GoalHashEntry->X &&
      newY == GoalHashEntry->Y &&
      newTheta == GoalHashEntry->Theta){
      OutHashEntry = GoalHashEntry;
    }
    else if((OutHashEntry = GetHashEntry(newX, newY, newTheta, newT, newInBubble)) == NULL){
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, newT, newInBubble);
		}
/*
    //if this is a 4D state (but didn't result from a wait motion primitive) then also output its 3D counterpart
    if(OutHashEntry->inBubble && aind != envDBubbleLatCfg.actionwidth-1){
      envDBubbleLatHashEntry_t* ExtraHashEntry;
      if((ExtraHashEntry = GetHashEntry(newX, newY, newTheta, newT, false)) == NULL)
        ExtraHashEntry = CreateNewHashEntry(newX, newY, newTheta, newT, false);
      //if(SourceStateID==340)
        //SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d bubble=%d h=%d cost=%d\n", ExtraHashEntry->stateID, newX, newY, newTheta, newT, ExtraHashEntry->inBubble, GetGoalHeuristic(ExtraHashEntry->stateID), cost);
      SuccIDV->push_back(ExtraHashEntry->stateID);
      CostV->push_back(cost);
      if(actionV != NULL)
        actionV->push_back(nav4daction);
    }
*/
    SuccIDV->push_back(OutHashEntry->stateID);
    //if(SourceStateID==9258)
    if(actionV != NULL)
      SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d bubble=%d h=%d cost=%d\n", OutHashEntry->stateID, newX, newY, newTheta, newT, OutHashEntry->inBubble, GetGoalHeuristic(OutHashEntry->stateID), cost);
    CostV->push_back(cost);
		if(actionV != NULL)
			actionV->push_back(nav4daction);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvDBubbleLat::Relax(int sourceID, int targetID){
	envDBubbleLatHashEntry_t* TargetEntry = StateID2CoordTable[targetID];
  if(TargetEntry->inBubble)
    return;
	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[sourceID];
  //don't try the last motion primitive since it is a "wait state" and is strictly 4D
	for (int aind = 0; aind < envDBubbleLatCfg.actionwidth-1; aind++){
  
		envDBubbleLatAction_t* nav4daction = &envDBubbleLatCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, ENVDBUBBLELAT_THETADIRS);	
    int newT = HashEntry->T + nav4daction->dT;

    if(GetHashEntry(newX, newY, newTheta, newT, false) == TargetEntry){
      if(newT < TargetEntry->T)
        TargetEntry->T = newT;
      return;
    }
  }
  //SBPL_PRINTF("WARNING: We tried to relax but can't find the transition from %d to %d\n", sourceID, targetID);
}

//TODO:Mike this function will not work, so we can't plan backwards because I can't compare to dynamic obstacles
//when I don't know how long it will take to get from the start to this state!!!!
void EnvDBubbleLat::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

  SBPL_PRINTF("Error: This environment currently does not support backward planning!\n");
  throw new SBPL_Exception();

	//TODO- to support tolerance, need: a) generate preds for goal state based on all possible goal state variable settings,
	//b) change goal check condition in gethashentry c) change getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(envDBubbleLatCfg.actionwidth); 
    CostV->reserve(envDBubbleLatCfg.actionwidth);

	//get X, Y for the state
	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == envDBubbleLatCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == envDBubbleLatCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < (int)envDBubbleLatCfg.PredActionsV[(int)HashEntry->Theta].size(); aind++)
	{

		envDBubbleLatAction_t* nav4daction = envDBubbleLatCfg.PredActionsV[(int)HashEntry->Theta].at(aind);

    int predX = HashEntry->X - nav4daction->dX;
		int predY = HashEntry->Y - nav4daction->dY;
		int predTheta = nav4daction->starttheta;	
    int predT = HashEntry->T - nav4daction->dT;
	
	
		//skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(predX, predY))
                continue;
        }

		//get cost
		int cost = GetActionCost(predX, predY, predTheta, predT, nav4daction);
	    if(cost >= INFINITECOST)
			continue;
        
    	envDBubbleLatHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY, predTheta, predT, false)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY, predTheta, predT, false);
		}

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}

void EnvDBubbleLat::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
		throw new SBPL_Exception();
	}

	if((int)state->Actions.size() != 0)
	{
		SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
		throw new SBPL_Exception();
	}
#endif
	

	//get X, Y for the state
	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];
	envDBubbleLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envDBubbleLat.goalstateid];

	//goal state should be absorbing
  if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta)
		return;
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == envDBubbleLatCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == envDBubbleLatCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < envDBubbleLatCfg.actionwidth; aind++)
	{
		envDBubbleLatAction_t* nav4daction = &envDBubbleLatCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, ENVDBUBBLELAT_THETADIRS);	
    int newT = HashEntry->T + nav4daction->dT;

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }

		//get cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav4daction);
        if(cost >= INFINITECOST)
            continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    	envDBubbleLatHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTheta, newT, false)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, newT, false);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}


void EnvDBubbleLat::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	envDBubbleLatHashEntry_t* affectedHashEntry;

	//increment iteration for processing savings
	iteration++;

	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedpredstatesV.size(); sind++)
		{
			affectedcell = affectedpredstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
		    affectedHashEntry = GetHashEntry(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.t, false);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvDBubbleLat::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	envDBubbleLatHashEntry_t* affectedHashEntry;

	SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
	throw new SBPL_Exception();

	//increment iteration for processing savings
	iteration++;

	//TODO - check
	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++)
		{
			affectedcell = affectedsuccstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
		    affectedHashEntry = GetHashEntry(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.t, false);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvDBubbleLat::InitializeEnvironment()
{
	envDBubbleLatHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	HashTableSize = 64*1024; //should be power of two
	Coord2StateIDHashTable = new vector<envDBubbleLatHashEntry_t*>[HashTableSize];
	
	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.StartY_c, envDBubbleLatCfg.StartTheta, envDBubbleLatCfg.StartTime, false);
	envDBubbleLat.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.EndY_c, envDBubbleLatCfg.EndTheta, INFINITECOST, false);
	envDBubbleLat.goalstateid = HashEntry->stateID;

	//initialized
	envDBubbleLat.bInitialized = true;

}


//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvDBubbleLat::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta, unsigned int T)
{

  return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)+(inthash(T)<<3)) & (HashTableSize-1);
}

void EnvDBubbleLat::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < HashTableSize; j++)
	{
	  if((int)Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

void EnvDBubbleLat::dumpStatesToFile(){
  FILE* fout = fopen("state_dump_bubble.txt", "w");
  for(unsigned int i=0; i<StateID2CoordTable.size(); i++){
    envDBubbleLatHashEntry_t* state = StateID2CoordTable[i];
    SBPL_FPRINTF(fout, "%d %d %d %d\n", state->X, state->Y, state->Theta, state->T);
  }
  fclose(fout);
}

int EnvDBubbleLat::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envDBubbleLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	//get X, Y for the state
	envDBubbleLatHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	envDBubbleLatHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	
	//TODO - check if one of the gridsearches already computed and then use it.
	

	return (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/envDBubbleLatCfg.nominalvel_mpersecs);	

}


int EnvDBubbleLat::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envDBubbleLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	if(bNeedtoRecomputeGoalHeuristics)
	{
		grid2Dsearchfromgoal->search(envDBubbleLatCfg.Grid2D, envDBubbleLatCfg.cost_inscribed_thresh, 
			envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.EndY_c, envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.StartY_c,  
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeGoalHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.StartY_c)
			/envDBubbleLatCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
	int hEuclid = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.EndY_c));
		

#if DEBUG
	SBPL_FPRINTF(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/envDBubbleLatCfg.nominalvel_mpersecs); 

}


int EnvDBubbleLat::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envDBubbleLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif


	if(bNeedtoRecomputeStartHeuristics)
	{
		grid2Dsearchfromstart->search(envDBubbleLatCfg.Grid2D, envDBubbleLatCfg.cost_inscribed_thresh, 
			envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.StartY_c, envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeStartHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(envDBubbleLatCfg.EndX_c, envDBubbleLatCfg.EndY_c)
			/envDBubbleLatCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	envDBubbleLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(ENVDBUBBLELAT_COSTMULT_MTOMM*EuclideanDistance_m(envDBubbleLatCfg.StartX_c, envDBubbleLatCfg.StartY_c, HashEntry->X, HashEntry->Y));
		

#if DEBUG
	SBPL_FPRINTF(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/envDBubbleLatCfg.nominalvel_mpersecs); 

}

int EnvDBubbleLat::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}
//------------------------------------------------------------------------------
