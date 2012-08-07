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
#include <sbpl_dynamic_planner/envInterval.h>
#include <algorithm>

#define DRAW_MAP 0

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0; //TODO-debugmax


//-----------------constructors/destructors-------------------------------
EnvIntervalLattice::EnvIntervalLattice()
{
  SBPL_PRINTF("u1\n");
	envIntervalLatCfg.obsthresh = ENVINTERVALLAT_DEFAULTOBSTHRESH;
	envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = ENVINTERVALLAT_DEFAULTDYNOBSTHRESH;
	envIntervalLatCfg.cost_inscribed_thresh = envIntervalLatCfg.obsthresh; //the value that pretty much makes it disabled
	envIntervalLatCfg.cost_possibly_circumscribed_thresh = -1; //the value that pretty much makes it disabled

	grid2Dsearchfromstart = NULL;
	grid2Dsearchfromgoal = NULL;
	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;
	iteration = 0;

	envIntervalLat.bInitialized = false;

	envIntervalLatCfg.actionwidth = ENVINTERVALLAT_DEFAULT_ACTIONWIDTH;
}

EnvIntervalLattice::~EnvIntervalLattice()
{
	if(grid2Dsearchfromstart != NULL)
		delete grid2Dsearchfromstart;
	grid2Dsearchfromstart = NULL;

	if(grid2Dsearchfromgoal != NULL)
		delete grid2Dsearchfromgoal;
	grid2Dsearchfromgoal = NULL;

  //free the environment
	for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++)
		delete [] envIntervalLatCfg.Grid2D[x];
	delete [] envIntervalLatCfg.Grid2D;

	for(int i=0; i<ENVINTERVALLAT_THETADIRS; i++)
		delete [] envIntervalLatCfg.ActionsV[i];  
	delete [] envIntervalLatCfg.ActionsV;

	for(int  i=0; i<ENVINTERVALLAT_THETADIRS; i++)
    envIntervalLatCfg.PredActionsV[i].clear();
	delete [] envIntervalLatCfg.PredActionsV;
}

EnvIntervalLat::~EnvIntervalLat(){
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

void EnvIntervalLattice::ReadDynamicObstacles(FILE* fDynObs){
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
    obs.radius = atof(sTemp) + envIntervalLatCfg.robotRadius;

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
        pt.t = CONTTIME2DISC(atof(sTemp),envIntervalLatCfg.timeResolution);

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

void EnvIntervalLattice::InitializeTimelineMap(){
  SBPL_PRINTF("Initializing interval and timeline maps\n");
  //create a clean intervalMap for dynamic obstacles
  intervalMap.resize(envIntervalLatCfg.EnvWidth_c);
  timelineMap.resize(envIntervalLatCfg.EnvWidth_c);
  for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++){
    intervalMap[x].resize(envIntervalLatCfg.EnvHeight_c);
    timelineMap[x].resize(envIntervalLatCfg.EnvHeight_c);
  }

  UpdateTimelineMap();
}

bool EnvIntervalLattice::UpdateTimelineMap(){
  SBPL_PRINTF("clearing interval map\n");
  //clear the bubble map
  for(unsigned int x=0; x<intervalMap.size(); x++){
    for(unsigned int y=0; y<intervalMap[x].size(); y++){
      intervalMap[x][y].clear();
      timelineMap[x][y].clear();
    }
  }

  SBPL_PRINTF("filling in new interval map\n");
  //fill in new interval map
  for(unsigned int i=0; i<dynamicObstacles.size(); i++){
    for(unsigned int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      //TODO:Mike handle trajectory probabilities for now I'll assume all probabilities are 1
      for(unsigned int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        //TODO:The +1 on the next line is a hack, to make sure all points on the continuous circle are contained in the discrete circle.
        //This makes sure we will not be in collision with a dynamic obstacle but it is too conservative and the padding will be to big
        //on some parts of the circle.
        int bubble_cell_rad = (int)ceil((dynamicObstacles[i].radius + p.std_dev*3)/envIntervalLatCfg.cellsize_m)+1;
        FillInBubble(CONTXY2DISC(p.x, envIntervalLatCfg.cellsize_m), CONTXY2DISC(p.y, envIntervalLatCfg.cellsize_m), p.t, bubble_cell_rad);
      }
    }
  }

  SBPL_PRINTF("creating timeline map\n");
  for(unsigned int x=0; x<timelineMap.size(); x++){
    for(unsigned int y=0; y<timelineMap[x].size(); y++){
      intervals2Timeline(x,y);
    }
  }

#if DRAW_MAP  
  SBPL_PRINTF("intervalMap\n");
  //draw bubble map
	for(int y = 0; y < envIntervalLatCfg.EnvHeight_c; y++){
		for(int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++){
      SBPL_PRINTF("(");
      for(unsigned int i=0; i<intervalMap[x][y].size(); i++)
        SBPL_PRINTF("%d %d,",intervalMap[x][y][i].first,intervalMap[x][y][i].second);
      SBPL_PRINTF(") ");
		}
    SBPL_PRINTF("\n");
  }
  SBPL_PRINTF("timelineMap\n");
  //draw bubble map
	for(int y = 0; y < envIntervalLatCfg.EnvHeight_c; y++){
		for(int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++){
      SBPL_PRINTF("(");
      for(unsigned int i=0; i<timelineMap[x][y].size(); i++)
        SBPL_PRINTF("%d,",timelineMap[x][y][i]);
      SBPL_PRINTF(") ");
		}
    SBPL_PRINTF("\n");
  }
#endif
  
  SBPL_PRINTF("done updating timeline map\n");
  //envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[envIntervalLat.startstateid];

	if(envIntervalLat.bInitialized){
    envIntervalLatHashEntry_t* HashEntry = getEntryFromID(envIntervalLat.startstateid);
    int interval = getInterval(HashEntry->X, HashEntry->Y, HashEntry->T);
    HashEntry->Interval = interval;
    return interval >= 0;
  }
  return true;
}

//Bresenham Circle Algorithm for finding edge of the circle so I can mark all cells inside
void EnvIntervalLattice::FillInBubble(int CenterX, int CenterY, int T, int rad){
  //SBPL_PRINTF("begin circle %d at (%d,%d,%d)  with radius %d\n",ID,CenterX,CenterY,T,rad);
  int d = 3 - 2*rad;
  int x = 0;
  int y = rad;

  while(x <= y){
    //mark pixels in 4 columns
    FillInBubbleColumn(CenterX-y, CenterY+x, CenterY-x, T);
    FillInBubbleColumn(CenterX-x, CenterY+y, CenterY-y, T);
    FillInBubbleColumn(CenterX+x, CenterY+y, CenterY-y, T);
    FillInBubbleColumn(CenterX+y, CenterY+x, CenterY-x, T);
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

void EnvIntervalLattice::FillInBubbleColumn(int x, int topY, int botY, int T){
  //SBPL_PRINTF("begin column %d from %d to %d\n", x, topY, botY);
  //return if column is outside map
  if(x < 0 || x >= (int)intervalMap.size())
    return;
  //snap topY and botY to edge of map if they are off of it
  if(topY >= (int)intervalMap[x].size())
    topY = intervalMap[x].size()-1;
  if(botY < 0)
    botY = 0;
  //return if column is outside map
  if(topY < 0 || botY >= (int)intervalMap[x].size())
    return;

  //SBPL_PRINTF("fill in top\n");
  //fill in top of column
  FillInBubbleCell(x, topY, T);
  
  //SBPL_PRINTF("fill in bottom\n");
  //fill in bottom of column
  FillInBubbleCell(x, botY, T);

  //SBPL_PRINTF("check if there is more column\n");
  //check if the rest of this column has already been filled in and if so we are done

  if(topY-botY <= 1)
    return;

  //otherwise fill in the column
  for(int y = topY-1; y>botY; y--){
    FillInBubbleCell(x, y, T);
  }

  //SBPL_PRINTF("end column\n");
}

void EnvIntervalLattice::FillInBubbleCell(int x, int y, int T){
  int minT = T - temporal_padding;
  int maxT = T + temporal_padding;
  intervalMap[x][y].push_back(pair<int,int>(minT,maxT));
}

void EnvIntervalLattice::intervals2Timeline(int x, int y){
  if(intervalMap[x][y].empty())
    return;
  sort(intervalMap[x][y].begin(), intervalMap[x][y].end(), pairCompare);
  timelineMap[x][y].push_back(intervalMap[x][y][0].first);
  timelineMap[x][y].push_back(intervalMap[x][y][0].second + 1);
  for(unsigned int i=1; i<intervalMap[x][y].size(); i++){
    if(intervalMap[x][y][i].first <= timelineMap[x][y].back() && intervalMap[x][y][i].second >= timelineMap[x][y].back())
      timelineMap[x][y].back() = intervalMap[x][y][i].second + 1;
    else if(intervalMap[x][y][i].first > timelineMap[x][y].back()){
      timelineMap[x][y].push_back(intervalMap[x][y][i].first);
      timelineMap[x][y].push_back(intervalMap[x][y][i].second + 1);
    }
  }
}

int EnvIntervalLattice::getInterval(int x, int y, int t){
  int i = distance(timelineMap[x][y].begin(), upper_bound(timelineMap[x][y].begin(), timelineMap[x][y].end(), t));
  if(i%2)
    return -1;
  return i/2;
}

void EnvIntervalLattice::getIntervals(vector<int>* intervals, vector<int>* times, envIntervalLatHashEntry_t* state, envIntervalLatAction_t* action){
  int newX = state->X + action->dX;
  int newY = state->Y + action->dY;
  int newT = state->T + action->dT;

  int state_interval = getInterval(state->X, state->Y, state->T);
  int last_leave_t = (timelineMap[state->X][state->Y].size() <= (unsigned int)(state_interval*2+1) ? INFINITECOST : timelineMap[state->X][state->Y][state_interval*2+1]-1);
  int last_newT = last_leave_t + action->dT;

  int firstInterval = distance(timelineMap[newX][newY].begin(), upper_bound(timelineMap[newX][newY].begin(), timelineMap[newX][newY].end(), newT));
  if(firstInterval%2)
    firstInterval++;
  firstInterval = firstInterval/2;
  int lastInterval = distance(timelineMap[newX][newY].begin(), upper_bound(timelineMap[newX][newY].begin(), timelineMap[newX][newY].end(), last_newT));
  if(lastInterval%2)
    lastInterval--;
  lastInterval = lastInterval/2;

  if(lastInterval < firstInterval)
    return;

  //SBPL_PRINTF("first interval=%d, last interval=%d\n",firstInterval,lastInterval);

  int t;
  if(lastInterval == firstInterval){
    t = getArrivalTimeToInterval(state, action, newT, last_newT);
    if(t >= 0){
      intervals->push_back(firstInterval);
      times->push_back(t);
    }
    return;
  }

  //handle the first interval
  t = getArrivalTimeToInterval(state, action, newT, timelineMap[newX][newY][firstInterval*2]-1);
  if(t >= 0){
    intervals->push_back(firstInterval);
    times->push_back(t);
  }

  //iterate over intervals between the first and last
  for(int i = firstInterval+1; i<lastInterval; i++){
    t = getArrivalTimeToInterval(state, action, timelineMap[newX][newY][i*2-1], timelineMap[newX][newY][i*2]-1);
    if(t >= 0){
      intervals->push_back(i);
      times->push_back(t);
    }
    //SBPL_PRINTF("check for bug here!!!!\n");
  }

  //handle the last interval
  t = getArrivalTimeToInterval(state, action, timelineMap[newX][newY][lastInterval*2-1], last_newT);
  if(t >= 0){
    intervals->push_back(lastInterval);
    times->push_back(t);
  }
}

//TODO: this is very inefficient....but I don't know how to improve it right now
int EnvIntervalLattice::getArrivalTimeToInterval(envIntervalLatHashEntry_t* state, envIntervalLatAction_t* action, int start_t, int end_t){
  for(int t=start_t - action->dT; t<=end_t - action->dT;){
    //SBPL_PRINTF("t=%d\n",t);
    bool collision = false;
    for(unsigned int i=0; i<action->interm4DcellsV.size(); i++){
      SBPL_4Dcell_t cell = action->interm4DcellsV.at(i);
      cell.x = cell.x + state->X;
      cell.y = cell.y + state->Y;
      cell.t = cell.t + t;
      //SBPL_PRINTF("x=%d y=%d t=%d\n",cell.x,cell.y,cell.t);

      int interval = distance(timelineMap[cell.x][cell.y].begin(), upper_bound(timelineMap[cell.x][cell.y].begin(), timelineMap[cell.x][cell.y].end(), cell.t));
      //SBPL_PRINTF("interval=%d\n",interval);
      if(interval%2){
        t += timelineMap[cell.x][cell.y][interval] - cell.t;
        collision = true;
        break;
      }
    }
    if(!collision)
      return t + action->dT;
  }
  return -1;
}

void EnvIntervalLattice::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta, int startTime,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double timeResolution, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
          const vector<sbpl_2Dpt_t> & robot_perimeterV) {
  envIntervalLatCfg.EnvWidth_c = width;
  envIntervalLatCfg.EnvHeight_c = height;
  envIntervalLatCfg.StartX_c = startx;
  envIntervalLatCfg.StartY_c = starty;
  envIntervalLatCfg.StartTheta = starttheta;
  envIntervalLatCfg.StartTime = startTime;
 
  if(envIntervalLatCfg.StartX_c < 0 || envIntervalLatCfg.StartX_c >= envIntervalLatCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(envIntervalLatCfg.StartY_c < 0 || envIntervalLatCfg.StartY_c >= envIntervalLatCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(envIntervalLatCfg.StartTheta < 0 || envIntervalLatCfg.StartTheta >= ENVINTERVALLAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
    throw new SBPL_Exception();
  }
  
  envIntervalLatCfg.EndX_c = goalx;
  envIntervalLatCfg.EndY_c = goaly;
  envIntervalLatCfg.EndTheta = goaltheta;

  if(envIntervalLatCfg.EndX_c < 0 || envIntervalLatCfg.EndX_c >= envIntervalLatCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(envIntervalLatCfg.EndY_c < 0 || envIntervalLatCfg.EndY_c >= envIntervalLatCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(envIntervalLatCfg.EndTheta < 0 || envIntervalLatCfg.EndTheta >= ENVINTERVALLAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
    throw new SBPL_Exception();
  }

  envIntervalLatCfg.FootprintPolygon = robot_perimeterV;

  envIntervalLatCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  envIntervalLatCfg.cellsize_m = cellsize_m;
  envIntervalLatCfg.timeResolution = timeResolution;
  envIntervalLatCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  envIntervalLatCfg.Grid2D = new unsigned char* [envIntervalLatCfg.EnvWidth_c];
  for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++) {
    envIntervalLatCfg.Grid2D[x] = new unsigned char [envIntervalLatCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < envIntervalLatCfg.EnvHeight_c; y++) {
      for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++) {
	envIntervalLatCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < envIntervalLatCfg.EnvHeight_c; y++) {
      for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++) {
			envIntervalLatCfg.Grid2D[x][y] = mapdata[x+y*width];
      }
    }
  }
}

void EnvIntervalLattice::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  envIntervalLatCfg structure
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
	envIntervalLatCfg.EnvWidth_c = atoi(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.EnvHeight_c = atoi(sTemp);

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
	envIntervalLatCfg.obsthresh = atoi(sTemp);
	SBPL_PRINTF("obsthresh = %d\n", envIntervalLatCfg.obsthresh);

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
	envIntervalLatCfg.cost_inscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_inscribed_thresh = %d\n", envIntervalLatCfg.cost_inscribed_thresh);


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
	envIntervalLatCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", envIntervalLatCfg.cost_possibly_circumscribed_thresh);

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
	envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = atoi(sTemp);
	SBPL_PRINTF("dynamic_obstacle_collision_cost_thresh = %d\n", envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh);

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
	envIntervalLatCfg.cellsize_m = atof(sTemp);
	
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
	envIntervalLatCfg.timeResolution = atof(sTemp);

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
	temporal_padding = (int)ceil(atof(sTemp)/envIntervalLatCfg.timeResolution);
  if(temporal_padding < 1)
    temporal_padding = 1;
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
	envIntervalLatCfg.nominalvel_mpersecs = atof(sTemp);
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
	envIntervalLatCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.StartX_c = CONTXY2DISC(atof(sTemp),envIntervalLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.StartY_c = CONTXY2DISC(atof(sTemp),envIntervalLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.StartTheta = ContTheta2Disc(atof(sTemp), ENVINTERVALLAT_THETADIRS);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.StartTime = CONTTIME2DISC(atof(sTemp),envIntervalLatCfg.timeResolution);


	if(envIntervalLatCfg.StartX_c < 0 || envIntervalLatCfg.StartX_c >= envIntervalLatCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(envIntervalLatCfg.StartY_c < 0 || envIntervalLatCfg.StartY_c >= envIntervalLatCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(envIntervalLatCfg.StartTheta < 0 || envIntervalLatCfg.StartTheta >= ENVINTERVALLAT_THETADIRS) {
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
	envIntervalLatCfg.EndX_c = CONTXY2DISC(atof(sTemp),envIntervalLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.EndY_c = CONTXY2DISC(atof(sTemp),envIntervalLatCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.EndTheta = ContTheta2Disc(atof(sTemp), ENVINTERVALLAT_THETADIRS);;

	if(envIntervalLatCfg.EndX_c < 0 || envIntervalLatCfg.EndX_c >= envIntervalLatCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(envIntervalLatCfg.EndY_c < 0 || envIntervalLatCfg.EndY_c >= envIntervalLatCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(envIntervalLatCfg.EndTheta < 0 || envIntervalLatCfg.EndTheta >= ENVINTERVALLAT_THETADIRS) {
		SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
		throw new SBPL_Exception();
	}


	//allocate the 2D environment
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	envIntervalLatCfg.Grid2D = new unsigned char* [envIntervalLatCfg.EnvWidth_c];
	for (x = 0; x < envIntervalLatCfg.EnvWidth_c; x++)
	{
		envIntervalLatCfg.Grid2D[x] = new unsigned char [envIntervalLatCfg.EnvHeight_c];
	}

	//environment:
	for (y = 0; y < envIntervalLatCfg.EnvHeight_c; y++){
		for (x = 0; x < envIntervalLatCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				SBPL_ERROR("ERROR: incorrect format of config file (%d,%d)\n",y,x);
				throw new SBPL_Exception();
			}
			//envIntervalLatCfg.Grid2D[x][y] = (dTemp >= envIntervalLatCfg.obsthresh ? 1 : 0);
			envIntervalLatCfg.Grid2D[x][y] = dTemp;
#if DRAW_MAP
      SBPL_PRINTF("%d ",envIntervalLatCfg.Grid2D[x][y]);
#endif
		}
#if DRAW_MAP
    SBPL_PRINTF("\n");
#endif
  }
  /*
  envIntervalLatCfg.obsthresh = 1;
	envIntervalLatCfg.cost_inscribed_thresh = 1;
	envIntervalLatCfg.cost_possibly_circumscribed_thresh = 0; 
	envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = 1;
  */

}

int EnvIntervalLattice::ReadinCell(SBPL_4Dcell_t* cell, char* fIn)
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
  cell->theta = NORMALIZEDISCTHETA(cell->theta, ENVINTERVALLAT_THETADIRS);
  
  //This is an optional param
  if((temp = strtok_r(NULL, " ", &sptr)) != NULL){
    cell->t = atoi(temp);
    return SUCCESS_WITH_TIME;
  }

  return SUCCESS_NO_TIME;
}

int EnvIntervalLattice::ReadinPose(SBPL_4Dpt_t* pose, char* fIn)
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

bool EnvIntervalLattice::ReadinMotionPrimitive(SBPL_xythetainterval_mprimitive* pMotPrim, FILE* fIn, bool &isWaitMotion)
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
    double timeFromTranslation = sqrt(x*x + y*y)/envIntervalLatCfg.nominalvel_mpersecs;
    double timeFromRotation = theta/(PI_CONST/4.0)*envIntervalLatCfg.timetoturn45degsinplace_secs;
    double t = __max(timeFromTranslation, timeFromRotation);
    if(t == 0)
      t = envIntervalLatCfg.timeResolution;

    pMotPrim->endcell.t = (int)ceil(t/envIntervalLatCfg.timeResolution);
    t = pMotPrim->endcell.t*envIntervalLatCfg.timeResolution;
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
	sourcepose.x = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
	sourcepose.y = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
	sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, ENVINTERVALLAT_THETADIRS);
  sourcepose.t = 0;
	double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
	double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
	double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;				
	double mp_endt = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].t;				
	int endx_c = CONTXY2DISC(mp_endx_m, envIntervalLatCfg.cellsize_m);
	int endy_c = CONTXY2DISC(mp_endy_m, envIntervalLatCfg.cellsize_m);
	int endtheta_c = ContTheta2Disc(mp_endtheta_rad, ENVINTERVALLAT_THETADIRS);
	int endt_c = CONTTIME2DISC(mp_endt, envIntervalLatCfg.timeResolution);
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



bool EnvIntervalLattice::ReadMotionPrimitives(FILE* fMotPrims)
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
  if(fabs(fTemp-envIntervalLatCfg.cellsize_m) > ERR_EPS){
    SBPL_ERROR("ERROR: invalid grid resolution %f (instead of %f) in the dynamics file\n", 
        fTemp, envIntervalLatCfg.cellsize_m);
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
  if(fabs(fTemp-envIntervalLatCfg.timeResolution) > ERR_EPS){
    SBPL_ERROR("ERROR: invalid time resolution %f (instead of %f) in the dynamics file\n", 
        fTemp, envIntervalLatCfg.timeResolution);
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
  if(dTemp != ENVINTERVALLAT_THETADIRS){
    SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", 
        dTemp, ENVINTERVALLAT_THETADIRS);
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
  waitMotFlags.resize(ENVINTERVALLAT_THETADIRS);
  for(int i = 0; i < ENVINTERVALLAT_THETADIRS; i++)
    waitMotFlags[i] = false;
  vector<SBPL_xythetainterval_mprimitive> waitMots;
  for(int i = 0; i < totalNumofActions; i++){
    SBPL_xythetainterval_mprimitive motprim;

    bool isWaitMotion;
    if(EnvIntervalLattice::ReadinMotionPrimitive(&motprim, fMotPrims, isWaitMotion) == false)
      return false;

    if(isWaitMotion){
      SBPL_PRINTF("WARN: wait motion in mprim file ignored\n");
    }
    else
      envIntervalLatCfg.mprimV.push_back(motprim);

  }
    
  SBPL_PRINTF("done ");

  return true;
}


void EnvIntervalLattice::ComputeReplanningDataforAction(envIntervalLatAction_t* action)
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
		endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVINTERVALLAT_THETADIRS); 
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
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVINTERVALLAT_THETADIRS); 
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
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, ENVINTERVALLAT_THETADIRS); 
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
void EnvIntervalLattice::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < ENVINTERVALLAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < envIntervalLatCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&envIntervalLatCfg.ActionsV[tind][aind]);
		}
	}
}

//here motionprimitivevector contains actions only for 0 angle
void EnvIntervalLattice::PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
	envIntervalLatCfg.ActionsV = new envIntervalLatAction_t* [ENVINTERVALLAT_THETADIRS];
	envIntervalLatCfg.PredActionsV = new vector<envIntervalLatAction_t*> [ENVINTERVALLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	//iterate over source angles
	for(int tind = 0; tind < ENVINTERVALLAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, ENVINTERVALLAT_THETADIRS);
		envIntervalLatCfg.ActionsV[tind] = new envIntervalLatAction_t[motionprimitiveV->size()];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVINTERVALLAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, envIntervalLatCfg.timeResolution);

		//iterate over motion primitives
		for(size_t aind = 0; aind < motionprimitiveV->size(); aind++)
		{
			envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;
			double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].x;
			double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].y;
			double mp_endtheta_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].theta;
			double mp_endt = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].t;
			
			double endx = sourcepose.x + (mp_endx_m*cos(sourcepose.theta) - mp_endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (mp_endx_m*sin(sourcepose.theta) + mp_endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, envIntervalLatCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, envIntervalLatCfg.cellsize_m);

			
			envIntervalLatCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad+sourcepose.theta, ENVINTERVALLAT_THETADIRS);
			envIntervalLatCfg.ActionsV[tind][aind].dX = endx_c;
			envIntervalLatCfg.ActionsV[tind][aind].dY = endy_c;
			envIntervalLatCfg.ActionsV[tind][aind].dT = CONTTIME2DISC(mp_endt + sourcepose.t, envIntervalLatCfg.timeResolution);
      /*
			if(envIntervalLatCfg.ActionsV[tind][aind].dY != 0 || envIntervalLatCfg.ActionsV[tind][aind].dX != 0) //if we translate
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.cellsize_m/envIntervalLatCfg.nominalvel_mpersecs*
								sqrt((double)(envIntervalLatCfg.ActionsV[tind][aind].dX*envIntervalLatCfg.ActionsV[tind][aind].dX + 
								envIntervalLatCfg.ActionsV[tind][aind].dY*envIntervalLatCfg.ActionsV[tind][aind].dY))));
			else if(envIntervalLatCfg.ActionsV[tind][aind].starttheta != envIntervalLatCfg.ActionsV[tind][aind].endtheta)//else if we turn in place
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*
						envIntervalLatCfg.timetoturn45degsinplace_secs*fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad,0))/(PI_CONST/4.0));
      else //else (we stand still)
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.ActionsV[tind][aind].dT*envIntervalLatCfg.timeResolution);
				//envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution));
      */
      envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.ActionsV[tind][aind].dT*envIntervalLatCfg.timeResolution);


			//compute and store interm points as well as intersecting cells
			envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
			envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.clear();
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
				envIntervalLatCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);

				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, envIntervalLatCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, envIntervalLatCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, ENVINTERVALLAT_THETADIRS); 
				interm4Dcell.t = CONTXY2DISC(pose.t, envIntervalLatCfg.timeResolution);
				if(envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;

			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f %.2f)\n",
				tind, aind, 			
				envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.ActionsV[tind][aind].dT,
				envIntervalLatCfg.ActionsV[tind][aind].endtheta, sourcepose.theta*180/PI_CONST, 
				envIntervalLatCfg.ActionsV[tind][aind].intermptV[envIntervalLatCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				envIntervalLatCfg.ActionsV[tind][aind].cost,
				mp_endx_m, mp_endy_m, mp_endt, mp_endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
			 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));

		}
	}

	//set number of actions
	envIntervalLatCfg.actionwidth = motionprimitiveV->size();


	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");


}


//here motionprimitivevector contains actions for all angles
void EnvIntervalLattice::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
	envIntervalLatCfg.ActionsV = new envIntervalLatAction_t* [ENVINTERVALLAT_THETADIRS];
	envIntervalLatCfg.PredActionsV = new vector<envIntervalLatAction_t*> [ENVINTERVALLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	if(motionprimitiveV->size()%ENVINTERVALLAT_THETADIRS != 0)
	{
		SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
		throw new SBPL_Exception();
	}

	envIntervalLatCfg.actionwidth = ((int)motionprimitiveV->size())/ENVINTERVALLAT_THETADIRS;

	//iterate over source angles
	int maxnumofactions = 0;
	for(int tind = 0; tind < ENVINTERVALLAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, ENVINTERVALLAT_THETADIRS);

		envIntervalLatCfg.ActionsV[tind] = new envIntervalLatAction_t[envIntervalLatCfg.actionwidth];  

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVINTERVALLAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, envIntervalLatCfg.timeResolution);


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
			envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;

			//compute dislocation
			envIntervalLatCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
			envIntervalLatCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
			envIntervalLatCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;
			envIntervalLatCfg.ActionsV[tind][aind].dT = motionprimitiveV->at(mind).endcell.t;

      /*
			//compute cost (if translating)
			if(envIntervalLatCfg.ActionsV[tind][aind].dY != 0 || envIntervalLatCfg.ActionsV[tind][aind].dX != 0)
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.cellsize_m/envIntervalLatCfg.nominalvel_mpersecs*
								sqrt((double)(envIntervalLatCfg.ActionsV[tind][aind].dX*envIntervalLatCfg.ActionsV[tind][aind].dX + 
								envIntervalLatCfg.ActionsV[tind][aind].dY*envIntervalLatCfg.ActionsV[tind][aind].dY))));
			else if(envIntervalLatCfg.ActionsV[tind][aind].starttheta != envIntervalLatCfg.ActionsV[tind][aind].endtheta)//cost (if turning in place)
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*
						envIntervalLatCfg.timetoturn45degsinplace_secs*
						fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS),
														DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].starttheta, ENVINTERVALLAT_THETADIRS)))/(PI_CONST/4.0));
      else //cost (if standing still)
				envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.ActionsV[tind][aind].dT*envIntervalLatCfg.timeResolution);
				//envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution));
      */
      envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.ActionsV[tind][aind].dT*envIntervalLatCfg.timeResolution);
      //SBPL_PRINTF("cost=%d\n",envIntervalLatCfg.ActionsV[tind][aind].cost);

			//use any additional cost multiplier
			//envIntervalLatCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

			//compute and store interm points as well as intersecting cells
			envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
			envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.clear();
			SBPL_4Dcell_t previnterm4Dcell;
			previnterm4Dcell.theta = 0; previnterm4Dcell.x = 0; previnterm4Dcell.y = previnterm4Dcell.t = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++)
			{
				SBPL_4Dpt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
		
				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				envIntervalLatCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);
			
				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, envIntervalLatCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, envIntervalLatCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, ENVINTERVALLAT_THETADIRS); 
				interm4Dcell.t = CONTTIME2DISC(pose.t, envIntervalLatCfg.timeResolution);
				if(envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					envIntervalLatCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;
			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprimID %d: %d %d %d %d)\n",
				tind, aind, 			
				envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.ActionsV[tind][aind].dT,
				envIntervalLatCfg.ActionsV[tind][aind].endtheta, 
				envIntervalLatCfg.ActionsV[tind][aind].intermptV[0].theta*180/PI_CONST, 
				envIntervalLatCfg.ActionsV[tind][aind].intermptV[envIntervalLatCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				envIntervalLatCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(mind).motprimID, 
				motionprimitiveV->at(mind).endcell.x, motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.t, motionprimitiveV->at(mind).endcell.theta);
#endif

			//add to the list of backward actions
			int targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
			 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));

		}

		if(maxnumofactions < numofactions)
			maxnumofactions = numofactions;
	}



	//at this point we don't allow nonuniform number of actions
	if(motionprimitiveV->size() != (size_t)(ENVINTERVALLAT_THETADIRS*maxnumofactions))
	{
		SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
				maxnumofactions, (int)motionprimitiveV->size(), ENVINTERVALLAT_THETADIRS);
		throw new SBPL_Exception();
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");

}

void EnvIntervalLattice::PrecomputeActions()
{
  SBPL_PRINTF("not supported (costs are not computed right)!\n");
  throw new SBPL_Exception();

	//construct list of actions
	SBPL_PRINTF("Pre-computing action data using the motion primitives for a 4D kinematic planning...\n");
	envIntervalLatCfg.ActionsV = new envIntervalLatAction_t* [ENVINTERVALLAT_THETADIRS];
	envIntervalLatCfg.PredActionsV = new vector<envIntervalLatAction_t*> [ENVINTERVALLAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < ENVINTERVALLAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("processing angle %d\n", tind);
		envIntervalLatCfg.ActionsV[tind] = new envIntervalLatAction_t[envIntervalLatCfg.actionwidth];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, envIntervalLatCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, ENVINTERVALLAT_THETADIRS);
		sourcepose.t = DISCTIME2CONT(0, envIntervalLatCfg.timeResolution);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;
			envIntervalLatCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1)%ENVINTERVALLAT_THETADIRS; //-1,0,1
			double angle = DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS);
			envIntervalLatCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			envIntervalLatCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
      //TODO:Mike we are ignoring the turn time in dT and in the cost, it is easily computable here 
      //and we know the robot rotates before moving so it seems like it should be added....
			envIntervalLatCfg.ActionsV[tind][aind].dT = (int)(ceil(envIntervalLatCfg.cellsize_m/envIntervalLatCfg.nominalvel_mpersecs*sqrt((double)(envIntervalLatCfg.ActionsV[tind][aind].dX*envIntervalLatCfg.ActionsV[tind][aind].dX + envIntervalLatCfg.ActionsV[tind][aind].dY*envIntervalLatCfg.ActionsV[tind][aind].dY))));
      envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ceil(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.cellsize_m/envIntervalLatCfg.nominalvel_mpersecs*sqrt((double)(envIntervalLatCfg.ActionsV[tind][aind].dX*envIntervalLatCfg.ActionsV[tind][aind].dX + envIntervalLatCfg.ActionsV[tind][aind].dY*envIntervalLatCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			SBPL_4Dpt_t pose;
			pose.x = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.cellsize_m);
			pose.y = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.cellsize_m);
			pose.theta = angle;
			pose.t = DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution);
			envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
			envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
				tind, aind, envIntervalLatCfg.ActionsV[tind][aind].endtheta, angle, 
				envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY,
				envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
			 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;
		envIntervalLatCfg.ActionsV[tind][aind].endtheta = tind-1; //TODO:Mike ask Max if this should be modulo the number of angle directions like below
		if(envIntervalLatCfg.ActionsV[tind][aind].endtheta < 0) envIntervalLatCfg.ActionsV[tind][aind].endtheta += ENVINTERVALLAT_THETADIRS;
		envIntervalLatCfg.ActionsV[tind][aind].dX = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dY = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dT = (int)(envIntervalLatCfg.timetoturn45degsinplace_secs);
		envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		SBPL_4Dpt_t pose;
		pose.x = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS);
		pose.t = DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution);
		envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
		envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envIntervalLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS),
			envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY,
			envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
		 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));


		aind = 4;
		envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;
		envIntervalLatCfg.ActionsV[tind][aind].endtheta = (tind + 1)%ENVINTERVALLAT_THETADIRS; 
		envIntervalLatCfg.ActionsV[tind][aind].dX = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dY = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dT = (int)(envIntervalLatCfg.timetoturn45degsinplace_secs);
		envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS);
		pose.t = DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution);
		envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
		envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envIntervalLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS),
			envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY,
			envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
		 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));

    //add one action for standing still
		aind = 5;
		envIntervalLatCfg.ActionsV[tind][aind].starttheta = tind;
		envIntervalLatCfg.ActionsV[tind][aind].endtheta = tind;
		envIntervalLatCfg.ActionsV[tind][aind].dX = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dY = 0;
		envIntervalLatCfg.ActionsV[tind][aind].dT = 1;
		envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*envIntervalLatCfg.ActionsV[tind][aind].dT*envIntervalLatCfg.timeResolution);
		//envIntervalLatCfg.ActionsV[tind][aind].cost = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution));

		//compute intersecting cells
		pose.x = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.cellsize_m);
		pose.y = DISCXY2CONT(envIntervalLatCfg.ActionsV[tind][aind].dY, envIntervalLatCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS);
		pose.t = DISCTIME2CONT(envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.timeResolution);
		envIntervalLatCfg.ActionsV[tind][aind].intermptV.clear();
		envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &envIntervalLatCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, envIntervalLatCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(envIntervalLatCfg.ActionsV[tind][aind].endtheta, ENVINTERVALLAT_THETADIRS),
			envIntervalLatCfg.ActionsV[tind][aind].dX, envIntervalLatCfg.ActionsV[tind][aind].dY,
			envIntervalLatCfg.ActionsV[tind][aind].dT, envIntervalLatCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = envIntervalLatCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + ENVINTERVALLAT_THETADIRS;
		 envIntervalLatCfg.PredActionsV[targettheta].push_back(&(envIntervalLatCfg.ActionsV[tind][aind]));
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data\n");


}



void EnvIntervalLattice::InitializeEnvConfig(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of envIntervalLatCfg if necessary

	//dXY dirs
	envIntervalLatCfg.dXY[0][0] = -1;
	envIntervalLatCfg.dXY[0][1] = -1;
	envIntervalLatCfg.dXY[1][0] = -1;
	envIntervalLatCfg.dXY[1][1] = 0;
	envIntervalLatCfg.dXY[2][0] = -1;
	envIntervalLatCfg.dXY[2][1] = 1;
	envIntervalLatCfg.dXY[3][0] = 0;
	envIntervalLatCfg.dXY[3][1] = -1;
	envIntervalLatCfg.dXY[4][0] = 0;
	envIntervalLatCfg.dXY[4][1] = 1;
	envIntervalLatCfg.dXY[5][0] = 1;
	envIntervalLatCfg.dXY[5][1] = -1;
	envIntervalLatCfg.dXY[6][0] = 1;
	envIntervalLatCfg.dXY[6][1] = 0;
	envIntervalLatCfg.dXY[7][0] = 1;
	envIntervalLatCfg.dXY[7][1] = 1;


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
			DISCXY2CONT(footprint.at(i).x, envIntervalLatCfg.cellsize_m), 
			DISCXY2CONT(footprint.at(i).y, envIntervalLatCfg.cellsize_m));
	}
#endif


	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);


}



bool EnvIntervalLattice::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < envIntervalLatCfg.EnvWidth_c && 
		Y >= 0 && Y < envIntervalLatCfg.EnvHeight_c && 
		envIntervalLatCfg.Grid2D[X][Y] < envIntervalLatCfg.obsthresh);
}

bool EnvIntervalLattice::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < envIntervalLatCfg.EnvWidth_c && 
		Y >= 0 && Y < envIntervalLatCfg.EnvHeight_c);
}

bool EnvIntervalLattice::IsValidConfiguration(int X, int Y, int Theta)
{
	vector<SBPL_4Dcell_t> footprint;
	SBPL_4Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, envIntervalLatCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, envIntervalLatCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, ENVINTERVALLAT_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (x < 0 || x >= envIntervalLatCfg.EnvWidth_c ||
			y < 0 || Y >= envIntervalLatCfg.EnvHeight_c ||		
			envIntervalLatCfg.Grid2D[x][y] >= envIntervalLatCfg.obsthresh)
		{
			return false;
		}
	}

	return true;
}


int EnvIntervalLattice::GetActionCost(int SourceX, int SourceY, int SourceTheta, envIntervalLatAction_t* action)
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
		
		if(interm4Dcell.x < 0 || interm4Dcell.x >= envIntervalLatCfg.EnvWidth_c ||
			interm4Dcell.y < 0 || interm4Dcell.y >= envIntervalLatCfg.EnvHeight_c)
			return INFINITECOST;

		maxcellcost = __max(maxcellcost, envIntervalLatCfg.Grid2D[interm4Dcell.x][interm4Dcell.y]);
		//check that the robot is NOT in the cell at which there is no valid orientation
		if(maxcellcost >= envIntervalLatCfg.cost_inscribed_thresh)
			return INFINITECOST;
	}

  /*
  if(SourceX==1 && SourceY==1 && SourceTheta==0)
    SBPL_PRINTF("2\n");
  if(SourceX==1 && SourceY==1 && SourceTheta==0)
    SBPL_PRINTF("maxcell=%d possibly=%d\n",maxcellcost,envIntervalLatCfg.cost_possibly_circumscribed_thresh);
    */

	//check collisions that for the particular footprint orientation along the action
	if(envIntervalLatCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= envIntervalLatCfg.cost_possibly_circumscribed_thresh)
	{
		checks++;

		for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
		{
			//get the cell in the map
			cell = action->intersectingcellsV.at(i);
			cell.x = cell.x + SourceX;
			cell.y = cell.y + SourceY;
			
			//check validity
			if(!IsValidCell(cell.x, cell.y)) {
        /*
        if(SourceX==1 && SourceY==1 && SourceTheta==0){
          for(int j = 0; j < (int)action->intersectingcellsV.size(); j++){
            SBPL_4Dcell_t cell2 = action->intersectingcellsV.at(j);
            SBPL_PRINTF("(%d,%d),",cell2.x,cell2.y);
          }
          SBPL_PRINTF("\n");
          SBPL_PRINTF("hmmm %d %d\n",cell.x,cell.y);
        }
        */
				return INFINITECOST;
      }

			//if(envIntervalLatCfg.Grid2D[cell.x][cell.y] > currentmaxcost) //cost computation changed: cost = max(cost of centers of the robot along action)
			//	currentmaxcost = envIntervalLatCfg.Grid2D[cell.x][cell.y];	//intersecting cells are only used for collision checking
		}
	}

  //if(SourceX==1 && SourceY==1 && SourceTheta==0)
    //SBPL_PRINTF("3\n");

	//to ensure consistency of h2D:
	//maxcellcost = __max(maxcellcost, envIntervalLatCfg.Grid2D[SourceX][SourceY]);
	//int currentmaxcost = (int)__max(maxcellcost, envIntervalLatCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

	return action->cost;//*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}

double EnvIntervalLattice::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return envIntervalLatCfg.cellsize_m*sqrt((double)sqdist);

}


//adds points to it (does not clear it beforehand)
void EnvIntervalLattice::CalculateFootprintForPose(SBPL_4Dpt_t pose, vector<SBPL_4Dcell_t>* footprint)
{  
	int pind;

#if DEBUG
//  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
//	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(envIntervalLatCfg.FootprintPolygon.size() <= 1){
    SBPL_4Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, envIntervalLatCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, envIntervalLatCfg.cellsize_m);

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
  for(find = 0; find < envIntervalLatCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = envIntervalLatCfg.FootprintPolygon[find];
    
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
  int prev_discrete_x = CONTXY2DISC(pt.x, envIntervalLatCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, envIntervalLatCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=envIntervalLatCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=envIntervalLatCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, envIntervalLatCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, envIntervalLatCfg.cellsize_m);
      
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


void EnvIntervalLattice::RemoveSourceFootprint(SBPL_4Dpt_t sourcepose, vector<SBPL_4Dcell_t>* footprint)
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

void EnvIntervalLattice::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	SBPL_PRINTF("Precomputing heuristics...\n");
	
	//allocated 2D grid searches
	grid2Dsearchfromstart = new SBPL2DGridSearch(envIntervalLatCfg.EnvWidth_c, envIntervalLatCfg.EnvHeight_c, (float)envIntervalLatCfg.cellsize_m);
	grid2Dsearchfromgoal = new SBPL2DGridSearch(envIntervalLatCfg.EnvWidth_c, envIntervalLatCfg.EnvHeight_c, (float)envIntervalLatCfg.cellsize_m);

	SBPL_PRINTF("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvIntervalLattice::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/ENVINTERVALLAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, ENVINTERVALLAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, ENVINTERVALLAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, ENVINTERVALLAT_THETADIRS);

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
bool EnvIntervalLattice::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sDynObsFile){
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
  return UpdateTimelineMap();
}

bool EnvIntervalLattice::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile)
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
  envIntervalLatCfg.robotRadius = sqrt(max_sqr_radius);
  SBPL_PRINTF("robotRadius=%f\n", envIntervalLatCfg.robotRadius);

	envIntervalLatCfg.FootprintPolygon = perimeterptsV;

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	ReadConfiguration(fCfg);
  fclose(fCfg);

  InitializeTimelineMap();

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
		InitGeneral(&envIntervalLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}


bool EnvIntervalLattice::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	ReadConfiguration(fCfg);
  fclose(fCfg);

	InitGeneral(NULL);


	return true;
}

bool EnvIntervalLattice::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution, double temporal_padding_c,
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


	envIntervalLatCfg.obsthresh = obsthresh;
	envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

  //Set up the dynamic obstacles...
  double max_sqr_radius = 0;
  for(unsigned int i=0; i<perimeterptsV.size(); i++){
    double x = perimeterptsV.at(i).x;
    double y = perimeterptsV.at(i).y;
    double d = x*x + y*y;
    if(d > max_sqr_radius)
      max_sqr_radius = d;
  }
  envIntervalLatCfg.robotRadius = sqrt(max_sqr_radius);
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + envIntervalLatCfg.robotRadius;
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
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, ENVINTERVALLAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, ENVINTERVALLAT_THETADIRS),
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
    fclose(fMotPrim);
	}

  temporal_padding = (int)ceil(temporal_padding_c/timeResolution);
  if(temporal_padding < 1)
    temporal_padding = 1;
  InitializeTimelineMap();

	if(envIntervalLatCfg.mprimV.size() != 0)
	{
		InitGeneral(&envIntervalLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}


bool EnvIntervalLattice::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution, double temporal_padding_c,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh, unsigned char dynobsthresh,  const char* sMotPrimFile)
{
  temporal_padding = (int)ceil(temporal_padding_c/timeResolution);
  if(temporal_padding < 1)
    temporal_padding = 1;

	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f timeResolution=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
		width, height, startx, starty, starttheta, startTime, goalx, goaly, goaltheta, cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	SBPL_PRINTF("perimeter has size=%d\n", (int)perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	envIntervalLatCfg.obsthresh = obsthresh;
	envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, ENVINTERVALLAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, ENVINTERVALLAT_THETADIRS),
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
    fclose(fMotPrim);
	}

	if(envIntervalLatCfg.mprimV.size() != 0)
	{
		InitGeneral(&envIntervalLatCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}


bool EnvIntervalLattice::InitGeneral(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);

  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvIntervalLattice::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = envIntervalLat.goalstateid;
	MDPCfg->startstateid = envIntervalLat.startstateid;

	return true;
}


void EnvIntervalLattice::PrintHeuristicValues()
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

		for (int y = 0; y < envIntervalLatCfg.EnvHeight_c; y++) {
			for (int x = 0; x < envIntervalLatCfg.EnvWidth_c; x++) {
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




void EnvIntervalLattice::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	SBPL_ERROR("ERROR in envIntervalLat... function: SetAllPreds is undefined\n");
	throw new SBPL_Exception();
}


void EnvIntervalLattice::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch)
{
  GetSuccs(SourceStateID, SuccIDV, CostV, OptV, optSearch, NULL);
}



const envIntervalLatConfig_t* EnvIntervalLattice::GetEnvNavConfig() {
  return &envIntervalLatCfg;
}



bool EnvIntervalLattice::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,envIntervalLatCfg.Grid2D[x][y], newcost);
#endif

    envIntervalLatCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

    return true;
}


void EnvIntervalLattice::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out envIntervalLat. configuration
	
	SBPL_ERROR("ERROR in envIntervalLat... function: PrintEnv_Config is undefined\n");
	throw new SBPL_Exception();

}

void EnvIntervalLattice::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}



bool EnvIntervalLattice::IsObstacle(int x, int y)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,envIntervalLatCfg.Grid2D[x][y]);
#endif


	return (envIntervalLatCfg.Grid2D[x][y] >= envIntervalLatCfg.obsthresh); 

}

int EnvIntervalLattice::getStartID(){
  return envIntervalLat.startstateid;
}

int EnvIntervalLattice::getGoalID(){
  return envIntervalLat.goalstateid;
}

void EnvIntervalLattice::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* startTime, double* goalx, double* goaly, double* goaltheta,
									  	double* cellsize_m, double* timeResolution, double* temporal_padding_, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, unsigned char* dyn_obs_thresh,
										vector<SBPL_xythetainterval_mprimitive>* mprimitiveV)
{
	*size_x = envIntervalLatCfg.EnvWidth_c;
	*size_y = envIntervalLatCfg.EnvHeight_c;

	*startx = DISCXY2CONT(envIntervalLatCfg.StartX_c, envIntervalLatCfg.cellsize_m);
	*starty = DISCXY2CONT(envIntervalLatCfg.StartY_c, envIntervalLatCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(envIntervalLatCfg.StartTheta, ENVINTERVALLAT_THETADIRS);
	*startTime = DISCTIME2CONT(envIntervalLatCfg.StartTime, envIntervalLatCfg.timeResolution);
	*goalx = DISCXY2CONT(envIntervalLatCfg.EndX_c, envIntervalLatCfg.cellsize_m);
	*goaly = DISCXY2CONT(envIntervalLatCfg.EndY_c, envIntervalLatCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(envIntervalLatCfg.EndTheta, ENVINTERVALLAT_THETADIRS);

	*cellsize_m = envIntervalLatCfg.cellsize_m;
	*timeResolution = envIntervalLatCfg.timeResolution;
  *temporal_padding_ = temporal_padding;
	*nominalvel_mpersecs = envIntervalLatCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = envIntervalLatCfg.timetoturn45degsinplace_secs;

	*dyn_obs_thresh = envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh;
	*obsthresh = envIntervalLatCfg.obsthresh;

	*mprimitiveV = envIntervalLatCfg.mprimV;
}

bool EnvIntervalLattice::PoseContToDisc(double px, double py, double pth, double pt,
					 int &ix, int &iy, int &ith, int &it) const
{
  ix = CONTXY2DISC(px, envIntervalLatCfg.cellsize_m);
  iy = CONTXY2DISC(py, envIntervalLatCfg.cellsize_m);
  ith = ContTheta2Disc(pth, ENVINTERVALLAT_THETADIRS); // ContTheta2Disc() normalizes the angle
  it = CONTTIME2DISC(pt, envIntervalLatCfg.timeResolution);
  return (pth >= -2*PI_CONST) && (pth <= 2*PI_CONST)
    && (ix >= 0) && (ix < envIntervalLatCfg.EnvWidth_c)
    && (iy >= 0) && (iy < envIntervalLatCfg.EnvHeight_c);
}

bool EnvIntervalLattice::PoseDiscToCont(int ix, int iy, int ith, int it,
					 double &px, double &py, double &pth, double &pt) const
{
  px = DISCXY2CONT(ix, envIntervalLatCfg.cellsize_m);
  py = DISCXY2CONT(iy, envIntervalLatCfg.cellsize_m);
  pth = normalizeAngle(DiscTheta2Cont(ith, ENVINTERVALLAT_THETADIRS));
  pt = DISCTIME2CONT(it, envIntervalLatCfg.timeResolution);
  return (ith >= 0) && (ith < ENVINTERVALLAT_THETADIRS)
    && (ix >= 0) && (ix < envIntervalLatCfg.EnvWidth_c)
    && (iy >= 0) && (iy < envIntervalLatCfg.EnvHeight_c);
}

unsigned char EnvIntervalLattice::GetMapCost(int x, int y)
{
	return envIntervalLatCfg.Grid2D[x][y];
}



bool EnvIntervalLattice::SetEnvParameter(const char* parameter, int value)
{

	if(envIntervalLat.bInitialized == true)
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
		envIntervalLatCfg.cost_inscribed_thresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envIntervalLatCfg.cost_possibly_circumscribed_thresh = value;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envIntervalLatCfg.obsthresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh = (unsigned char)value;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		return false;
	}

	return true;
}

int EnvIntervalLattice::GetEnvParameter(const char* parameter)
{

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		return (int) envIntervalLatCfg.cost_inscribed_thresh;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		return (int) envIntervalLatCfg.cost_possibly_circumscribed_thresh;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		return (int) envIntervalLatCfg.obsthresh;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		return (int) envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		throw new SBPL_Exception();
	}

}

//------------------------------------------------------------------------------



//-----------------XYTHETA Enivornment (child) class---------------------------
void EnvIntervalLat::GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t) const {
  envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  t = HashEntry->T;
}
void EnvIntervalLat::GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t, bool& opt) const {
  envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  t = HashEntry->T;
  opt = HashEntry->Optimal;
}

int EnvIntervalLat::GetStateFromCoord(int x, int y, int theta, int t){
  SBPL_PRINTF("Error: GetStateFromCoord shouldn't be used right now...\n");
  //throw new SBPL_Exception();

  envIntervalLatHashEntry_t* OutHashEntry;
  if((OutHashEntry = GetHashEntry(x, y, theta, getInterval(x,y,t), t, true)) == NULL){
    if((OutHashEntry = GetHashEntry(x, y, theta, getInterval(x,y,t), t, false)) == NULL){
      //have to create a new entry
      OutHashEntry = CreateNewHashEntry(x, y, theta, getInterval(x,y,t), t, true);
    }
  }
  SBPL_PRINTF("interval = %d\n",getInterval(x,y,t));
  for(unsigned int i=0; i<timelineMap[x][y].size(); i++)
      SBPL_PRINTF("%d ",timelineMap[x][y][i]);
  SBPL_PRINTF("\n");
  return OutHashEntry->stateID;
}

void EnvIntervalLat::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath)
{
	vector<envIntervalLatAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
  vector<bool> OptV;
	int targetx_c, targety_c, targettheta_c;
	int sourcex_c, sourcey_c, sourcetheta_c;
  int targett_c, sourcet_c;
  bool source_opt, target_opt;

	//SBPL_PRINTF("checks=%ld\n", checks);

	xythetaPath->clear();

#if DEBUG
	SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

	for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
	{
		int sourceID = stateIDPath->at(pind);
		int targetID = stateIDPath->at(pind+1);

#if DEBUG
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
#endif


		//get successors and pick the target via the cheapest action
		SuccIDV.clear();
		CostV.clear();
		actionV.clear();
		OptV.clear();
		GetSuccs(sourceID, &SuccIDV, &CostV, &OptV, false, &actionV);
		
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
			GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, source_opt);
			GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c, target_opt);
			SBPL_PRINTF("ID=%d %d %d %d %d %d -> ID=%d %d %d %d %d %d\n", sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, source_opt,
					targetID, targetx_c, targety_c, targettheta_c, targett_c, target_opt); 
			throw new SBPL_Exception();
		}

		//now push in the actual path
		int sourcex_c, sourcey_c, sourcetheta_c, sourcet_c;
		int targetx_c, targety_c, targettheta_c, targett_c;
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, source_opt);
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c, target_opt);
		double sourcex, sourcey, sourcet;
		sourcex = DISCXY2CONT(sourcex_c, envIntervalLatCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, envIntervalLatCfg.cellsize_m);

    if(targett_c - sourcet_c > actionV[bestsind]->dT){
			SBPL_4Dpt_t intermpt = actionV[bestsind]->intermptV[0];
      intermpt.x += sourcex;
      intermpt.y += sourcey;
      intermpt.t = DISCTIME2CONT(sourcet_c, envIntervalLatCfg.timeResolution);
			xythetaPath->push_back(intermpt);
      sourcet = DISCTIME2CONT(targett_c-actionV[bestsind]->dT, envIntervalLatCfg.timeResolution);
      //SBPL_PRINTF("x=%f y=%f th=%f t=%f; t0=%f\n",intermpt.x,intermpt.y,intermpt.theta,intermpt.t,sourcet);
      SBPL_PRINTF("post-process wait %d\n", targett_c-sourcet_c-actionV[bestsind]->dT);
    }
    else
      sourcet = DISCTIME2CONT(sourcet_c, envIntervalLatCfg.timeResolution);

		//TODO - when there are no motion primitives we should still print source state
		for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
		{
			//translate appropriately
			SBPL_4Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
			intermpt.x += sourcex;
			intermpt.y += sourcey;
			intermpt.t += sourcet;

#if DEBUG
			int nx = CONTXY2DISC(intermpt.x, envIntervalLatCfg.cellsize_m);
			int ny = CONTXY2DISC(intermpt.y, envIntervalLatCfg.cellsize_m);
			int nt = CONTTIME2DISC(intermpt.t, envIntervalLatCfg.timeResolution);
			SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f %.3f (%d %d %d cost=%d) ", 
				intermpt.x, intermpt.y, intermpt.theta, intermpt.t
				nx, ny, 
				ContTheta2Disc(intermpt.theta, ENVINTERVALLAT_THETADIRS), nt, envIntervalLatCfg.Grid2D[nx][ny]);
			if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
			else SBPL_FPRINTF(fDeb, "\n");
#endif

			//store
			xythetaPath->push_back(intermpt);
		}
	}
}


//returns the stateid if success, and -1 otherwise
int EnvIntervalLat::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, envIntervalLatCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, envIntervalLatCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, ENVINTERVALLAT_THETADIRS);

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

    envIntervalLatHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, INFINITECOST, INFINITECOST, false)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, INFINITECOST, INFINITECOST, false);
    }

	//need to recompute start heuristics?
	//if(envIntervalLat.goalstateid != OutHashEntry->stateID)
  int oldGoalX, oldGoalY, oldGoalTheta;
  int oldGoalT;
  bool oldGoalOpt;
  GetCoordFromState(envIntervalLat.goalstateid, oldGoalX, oldGoalY, oldGoalTheta, oldGoalT, oldGoalOpt);
  if(oldGoalX != x || oldGoalY != y || oldGoalTheta != theta)
	{
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
		bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
	}



    envIntervalLat.goalstateid = OutHashEntry->stateID;

	envIntervalLatCfg.EndX_c = x;
	envIntervalLatCfg.EndY_c = y;
	envIntervalLatCfg.EndTheta = theta;

  
  //dumpEnvironmentToFile();
  //dumpDynamicObstaclesToFile();
  //throw new SBPL_Exception();
  

    return envIntervalLat.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvIntervalLat::SetStart(double x_m, double y_m, double theta_rad, double startTime){

	int x = CONTXY2DISC(x_m, envIntervalLatCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, envIntervalLatCfg.cellsize_m); 
	int theta = ContTheta2Disc(theta_rad, ENVINTERVALLAT_THETADIRS);
  int t = CONTTIME2DISC(startTime, envIntervalLatCfg.timeResolution);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
		return -1;
	}

	SBPL_PRINTF("env: setting start to %.3f %.3f %.3f %.3f (%d %d %d %d)\n", x_m, y_m, theta_rad, startTime, x, y, theta, t);

    if(!IsValidConfiguration(x,y,theta) || getInterval(x,y,t) < 0)
	{
		SBPL_PRINTF("WARNING: start configuration x=%d y=%d th=%d t=%d i=%d is invalid\n", x,y,theta,t,getInterval(x,y,t));
   	        return -1;
	}

    envIntervalLatHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, getInterval(x,y,t), t, true)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, getInterval(x,y,t), t, true);
    }

	//need to recompute start heuristics?
	//if(envIntervalLat.startstateid != OutHashEntry->stateID)
  int oldStartX, oldStartY, oldStartTheta;
  int oldStartT;
  bool oldStartOpt;
  GetCoordFromState(envIntervalLat.startstateid, oldStartX, oldStartY, oldStartTheta, oldStartT, oldStartOpt);
  if(oldStartX != x || oldStartY != y || oldStartTheta != theta)
	{
		bNeedtoRecomputeStartHeuristics = true;
		bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
	}

	//set start
    envIntervalLat.startstateid = OutHashEntry->stateID;
	envIntervalLatCfg.StartX_c = x;
	envIntervalLatCfg.StartY_c = y;
	envIntervalLatCfg.StartTheta = theta;
	envIntervalLatCfg.StartTime = t;

    return envIntervalLat.startstateid;    

}

bool EnvIntervalLat::setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states /*=true*/){
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + envIntervalLatCfg.robotRadius;
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
        //SBPL_PRINTF("%f %f %f %f\n",p.x,p.y,DISCTIME2CONT(p.t, envIntervalLatCfg.timeResolution),p.std_dev);
      }
      obs.trajectories.push_back(traj);
    }
    dynamicObstacles.push_back(obs);
  }
  SBPL_PRINTF("we have %d dynamic obstacles\n", (int)dynamicObstacles.size());
  //SBPL_Traj_Pt_t p1 = dynamicObstacles[0].trajectories[0].points[0];
  //SBPL_Traj_Pt_t p2 = dynamicObstacles[0].trajectories[0].points[dynamicObstacles[0].trajectories[0].points.size()-1];
  //SBPL_PRINTF("start=(%f, %f, %d) end=(%f, %f, %d)\n", p1.x, p1.y, p1.t, p2.x, p2.y, p2.t);

  bool ret = UpdateTimelineMap();
  if(reset_states){
    ClearStates();
    return true;
  }
  return ret;
}

void EnvIntervalLat::ClearStates(){
  SBPL_PRINTF("clearing states...\n");
	for(unsigned int i=0; i<StateID2CoordTable.size(); i++)
    StateID2CoordTable[i]->init = false;
    //StateID2CoordTable[i]->T = -1;
  /*
    delete StateID2CoordTable[i];
  StateID2CoordTable.clear();
    
	HashTableSize = 64*1024; //should be the same as the other HashTableSize
  for(int i=0; i<HashTableSize; i++)
    Coord2StateIDHashTable[i].clear();

  for(unsigned int i=0; i<StateID2IndexMapping.size(); i++)
    delete [] StateID2IndexMapping[i];
  StateID2IndexMapping.clear();
  */
  SBPL_PRINTF("done clearing states\n");
}

envIntervalLatHashEntry_t* EnvIntervalLat::getEntryFromID(int id){
  return StateID2CoordTable[id];
}

void EnvIntervalLat::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envIntervalLat... function: stateID illegal (2)\n");
		throw new SBPL_Exception();
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	envIntervalLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envIntervalLat.goalstateid];

	if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta && bVerbose)
	{
		SBPL_FPRINTF(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d T=%d Opt=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, HashEntry->Optimal);
    else
    	SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, envIntervalLatCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,envIntervalLatCfg.cellsize_m), 
		DiscTheta2Cont(HashEntry->Theta, ENVINTERVALLAT_THETADIRS), DISCTIME2CONT(HashEntry->T, envIntervalLatCfg.timeResolution));

}


envIntervalLatHashEntry_t* EnvIntervalLat::GetHashEntry(int X, int Y, int Theta, int interval, int T, bool Opt)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

  if(interval < 0){
    SBPL_ERROR("ERROR: tried to get a Hash Entry (x=%d y=%d theta=%d interval=%d) in a collision interval!\n", X,Y,Theta,interval);
    throw new SBPL_Exception();
  }
  
  int binid = GETHASHBIN(X, Y, Theta, interval, Opt);
	
#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 500)
	{
		SBPL_PRINTF("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)Coord2StateIDHashTable[binid].size(); ind++)
  {
    if( Coord2StateIDHashTable[binid][ind]->X == X 
      && Coord2StateIDHashTable[binid][ind]->Y == Y
      && Coord2StateIDHashTable[binid][ind]->Theta == Theta
      && Coord2StateIDHashTable[binid][ind]->Interval == interval
      && Coord2StateIDHashTable[binid][ind]->Optimal == Opt)
    {
#if TIME_DEBUG
      time_gethash += clock()-currenttime;
#endif
      //if(Coord2StateIDHashTable[binid][ind]->T < 0){
      if(!Coord2StateIDHashTable[binid][ind]->init){
        Coord2StateIDHashTable[binid][ind]->T = T;
        Coord2StateIDHashTable[binid][ind]->tempT = T;
        Coord2StateIDHashTable[binid][ind]->init = true;
      }
      return Coord2StateIDHashTable[binid][ind];
    }
  }

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


envIntervalLatHashEntry_t* EnvIntervalLat::CreateNewHashEntry(int X, int Y, int Theta, int interval, int T, bool Opt) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

  if(interval < 0){
    SBPL_ERROR("ERROR: tried to create a Hash Entry (x=%d y=%d theta=%d interval=%d t=%d) in a collision interval!\n", X,Y,Theta,interval,T);
    throw new SBPL_Exception();
  }

	envIntervalLatHashEntry_t* HashEntry = new envIntervalLatHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
  HashEntry->T = T;
  HashEntry->tempT = T;
  HashEntry->Interval = interval;
  HashEntry->Optimal = Opt;
	HashEntry->iteration = 0;
  HashEntry->closed = false;
  HashEntry->init = true;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
  i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->Interval, HashEntry->Optimal); 
    

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

void EnvIntervalLat::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch, vector<envIntervalLatAction_t*>* actionV /*=NULL*/)
{
  bool print_expand = false;
  //bool print_expand = SourceStateID == 9695 || SourceStateID == 9227;
  //bool print_expand = actionV != NULL;
  int aind;

  if(print_expand)
    SBPL_PRINTF("optimal search=%d\n",optSearch);

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(envIntervalLatCfg.actionwidth); 
  CostV->reserve(envIntervalLatCfg.actionwidth);
	if(actionV != NULL)
	{
		actionV->clear();
		actionV->reserve(envIntervalLatCfg.actionwidth);
	}

	//get X, Y for the state
	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  HashEntry->closed = true;
	envIntervalLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envIntervalLat.goalstateid];
	
  if(print_expand)
    SBPL_PRINTF("stateid=%d x=%d, y=%d, theta=%d, t=%d int=%d opt=%d\n",SourceStateID,HashEntry->X,HashEntry->Y,HashEntry->Theta,HashEntry->T,HashEntry->Interval,HashEntry->Optimal);

  if(SourceStateID == envIntervalLat.goalstateid){
    return;
  }

	//iterate through actions
	for (aind = 0; aind < envIntervalLatCfg.actionwidth; aind++){
    
    //compute new pose from this aind action (successor pose)
    envIntervalLatAction_t* nav4daction = &envIntervalLatCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
    int newY = HashEntry->Y + nav4daction->dY;
    int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, ENVINTERVALLAT_THETADIRS);	

    //if(SourceStateID==0)
      //SBPL_PRINTF("      x=%d y=%d theta=%d\n", newX, newY, newTheta);

    //skip the invalid cells
    if(!IsValidCell(newX, newY)) 
      continue;

    //if(SourceStateID==0)
      //SBPL_PRINTF("      valid\n");

    //get cost
    int base_cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav4daction);
    if(base_cost >= INFINITECOST)
      continue;

    //if(SourceStateID==0)
      //SBPL_PRINTF("      base_cost=%d\n", base_cost);

    //int initialT = HashEntry->T + nav4daction->dT;
    vector<int> intervals;
    vector<int> times;
    getIntervals(&intervals, &times, HashEntry, nav4daction);
    //if(intervals.size() > 1)
      //done = true;

    for(unsigned int i=0; i<intervals.size(); i++){
      envIntervalLatHashEntry_t* OutHashEntry;
      int cost = base_cost + (int)(ENVINTERVALLAT_COSTMULT_MTOMM*(times[i]-HashEntry->T-nav4daction->dT)*envIntervalLatCfg.timeResolution);

      //TODO:Mike let this have some close enough threshold
      if(newX == GoalHashEntry->X && newY == GoalHashEntry->Y && newTheta == GoalHashEntry->Theta){
        OutHashEntry = GoalHashEntry;
        if(times[i] < OutHashEntry->tempT)
          OutHashEntry->tempT = times[i];
        if(intervals[i] < OutHashEntry->Interval)
          OutHashEntry->Interval = intervals[i];
        SBPL_PRINTF("found goal at t=%d\n",OutHashEntry->tempT);
        if(print_expand)
          SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d int=%d opt=%d h=%d cost=%d\n", OutHashEntry->stateID, newX, newY, newTheta, OutHashEntry->tempT, OutHashEntry->Interval, OutHashEntry->Optimal, GetGoalHeuristic(OutHashEntry->stateID), cost);

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        OptV->push_back(false);
        if(actionV != NULL)
          actionV->push_back(nav4daction);
      }
      else{
        if(!optSearch){
          if((OutHashEntry = GetHashEntry(newX, newY, newTheta, intervals[i], times[i], false)) == NULL){
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, intervals[i], times[i], false);
          }
          //TODO: this won't work if epsilon > 1 since we could expand a state that is not optimal (not earliest time) and later discover the state at a cheaper cost
          //then we will lower the time of the HashEntry (but won't expand state due to CLOSED list) but when we reconstruct the path the times will be off
          //put relax function back in for when epsilon > 1
          //OR check in this else if that OutHashEntry hasn't been CLOSED yet (we can keep track of closed states by putting a flag in the HashEntry and setting it
          //to true at the end of this getSuccs function)
          else if(times[i] < OutHashEntry->tempT)
            OutHashEntry->tempT = times[i];

          //put in the sub-optimal state
          SuccIDV->push_back(OutHashEntry->stateID);
          CostV->push_back(cost);
          OptV->push_back(false);
          if(actionV != NULL)
            actionV->push_back(nav4daction);
          if(print_expand)
            SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d int=%d opt=%d h=%d cost=%d\n", OutHashEntry->stateID, newX, newY, newTheta, OutHashEntry->tempT, OutHashEntry->Interval, OutHashEntry->Optimal, GetGoalHeuristic(OutHashEntry->stateID), cost);
        }

        //add the optimal version if the parent was optimal
        if(HashEntry->Optimal){
          if((OutHashEntry = GetHashEntry(newX, newY, newTheta, intervals[i], times[i], true)) == NULL)
            OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, intervals[i], times[i], true);
          else if(times[i] < OutHashEntry->tempT)
            OutHashEntry->tempT = times[i];
          SuccIDV->push_back(OutHashEntry->stateID);
          CostV->push_back(cost);
          OptV->push_back(true);
          if(actionV != NULL)
            actionV->push_back(nav4daction);
          if(print_expand)
            SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d int=%d opt=%d h=%d cost=%d\n", OutHashEntry->stateID, newX, newY, newTheta, OutHashEntry->tempT, OutHashEntry->Interval, OutHashEntry->Optimal, GetGoalHeuristic(OutHashEntry->stateID), cost);
        }
      }
    }
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif
}

void EnvIntervalLat::Relax(int stateID){
	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  HashEntry->T = HashEntry->tempT;
}

//TODO:Mike this function will not work, so we can't plan backwards because I can't compare to dynamic obstacles
//when I don't know how long it will take to get from the start to this state!!!!
void EnvIntervalLat::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
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
    PredIDV->reserve(envIntervalLatCfg.actionwidth); 
    CostV->reserve(envIntervalLatCfg.actionwidth);

	//get X, Y for the state
	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == envIntervalLatCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == envIntervalLatCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < (int)envIntervalLatCfg.PredActionsV[(int)HashEntry->Theta].size(); aind++)
	{

		envIntervalLatAction_t* nav4daction = envIntervalLatCfg.PredActionsV[(int)HashEntry->Theta].at(aind);

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
		int cost = GetActionCost(predX, predY, predTheta, nav4daction);
	    if(cost >= INFINITECOST)
			continue;
        
    	envIntervalLatHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY, predTheta, getInterval(predX, predY, predT), predT, false)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY, predTheta, getInterval(predX, predY, predT), predT, false);
		}

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}

void EnvIntervalLat::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR: this function is not ready for use!\n");
  throw new SBPL_Exception();

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
	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];
	envIntervalLatHashEntry_t* GoalHashEntry = StateID2CoordTable[envIntervalLat.goalstateid];

	//goal state should be absorbing
  if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta)
		return;
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == envIntervalLatCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == envIntervalLatCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < envIntervalLatCfg.actionwidth; aind++)
	{
		envIntervalLatAction_t* nav4daction = &envIntervalLatCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, ENVINTERVALLAT_THETADIRS);	
    int newT = HashEntry->T + nav4daction->dT;

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }

		//get cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav4daction);
        if(cost >= INFINITECOST)
            continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    	envIntervalLatHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTheta, getInterval(newX, newY, newT), newT,false)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, getInterval(newX, newY, newT), newT,false);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}


void EnvIntervalLat::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
  SBPL_ERROR("ERROR: this function is not ready for use!\n");
  throw new SBPL_Exception();
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	envIntervalLatHashEntry_t* affectedHashEntry;

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
		    affectedHashEntry = GetHashEntry(affectedcell.x, affectedcell.y, affectedcell.theta, getInterval(affectedcell.x, affectedcell.y, affectedcell.t), affectedcell.t, false);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvIntervalLat::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	envIntervalLatHashEntry_t* affectedHashEntry;

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
		    affectedHashEntry = GetHashEntry(affectedcell.x, affectedcell.y, affectedcell.theta, getInterval(affectedcell.x, affectedcell.y, affectedcell.t), affectedcell.t, false);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvIntervalLat::InitializeEnvironment()
{
	envIntervalLatHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	HashTableSize = 64*1024; //should be power of two
	Coord2StateIDHashTable = new vector<envIntervalLatHashEntry_t*>[HashTableSize];
	
	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
  if(getInterval(envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, envIntervalLatCfg.StartTime) < 0){
		SBPL_PRINTF("WARNING: start time (%d %d %d) is invalid\n", envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, envIntervalLatCfg.StartTime);
	}
	HashEntry = CreateNewHashEntry(envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, envIntervalLatCfg.StartTheta, getInterval(envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, envIntervalLatCfg.StartTime), envIntervalLatCfg.StartTime, true);
	envIntervalLat.startstateid = HashEntry->stateID;
  SBPL_PRINTF("id=%d X=%d Y=%d Theta=%d T=%d\n", HashEntry->stateID, HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T);

	//create goal state 
	HashEntry = CreateNewHashEntry(envIntervalLatCfg.EndX_c, envIntervalLatCfg.EndY_c, envIntervalLatCfg.EndTheta, INFINITECOST, INFINITECOST, false);
	envIntervalLat.goalstateid = HashEntry->stateID;

	//initialized
	envIntervalLat.bInitialized = true;

}


//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvIntervalLat::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta, unsigned int interval, bool Opt)
{

  return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)+(inthash(interval)<<3)) & (HashTableSize-1);
}

void EnvIntervalLat::PrintHashTableHist()
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

void EnvIntervalLat::getExpansions(vector<SBPL_4Dpt_t>* points){
  for(unsigned int i=0; i<StateID2CoordTable.size(); i++){
    envIntervalLatHashEntry_t* state = StateID2CoordTable[i];
    if(state->closed){
      SBPL_4Dpt_t p;
      p.x = DISCXY2CONT(state->X, envIntervalLatCfg.cellsize_m);
      p.y = DISCXY2CONT(state->Y, envIntervalLatCfg.cellsize_m);
      p.theta = DISCXY2CONT(state->Theta, ENVINTERVALLAT_THETADIRS);
      p.t = DISCTIME2CONT(state->T, envIntervalLatCfg.timeResolution);
      points->push_back(p);
    }
  }
}

void EnvIntervalLat::dumpStatesToFile(){
  FILE* fout = fopen("state_dump_interval.txt", "w");
  for(unsigned int i=0; i<StateID2CoordTable.size(); i++){
    envIntervalLatHashEntry_t* state = StateID2CoordTable[i];
    //SBPL_FPRINTF(fout, "x=%d y=%d th=%d i=%d t=%d\n", state->X, state->Y, state->Theta, state->Interval, state->T);
    SBPL_FPRINTF(fout, "%d %d %d %d %d %d\n", state->X, state->Y, state->Theta, state->Interval, state->T, state->Optimal);
  }
  fclose(fout);
}

void EnvIntervalLat::dumpEnvironmentToFile(){
  FILE* fout = fopen("hall.cfg", "w");
  SBPL_FPRINTF(fout, "discretization(cells): %d %d\n", envIntervalLatCfg.EnvWidth_c, envIntervalLatCfg.EnvHeight_c);
  SBPL_FPRINTF(fout, "obsthresh: %d\n", envIntervalLatCfg.obsthresh);
  SBPL_FPRINTF(fout, "cost_inscribed_thresh: %d\n", envIntervalLatCfg.cost_inscribed_thresh);
  SBPL_FPRINTF(fout, "cost_possibly_circumscribed_thresh: %d\n", envIntervalLatCfg.cost_possibly_circumscribed_thresh);
  SBPL_FPRINTF(fout, "dynamic_obstacle_collision_cost_thresh: %d\n", envIntervalLatCfg.dynamic_obstacle_collision_cost_thresh);
  SBPL_FPRINTF(fout, "cellsize(meters): %f\n", envIntervalLatCfg.cellsize_m);
  SBPL_FPRINTF(fout, "timeResolution(seconds): %f\n", envIntervalLatCfg.timeResolution);
  SBPL_FPRINTF(fout, "nominalvel(mpersecs): %f\n", envIntervalLatCfg.nominalvel_mpersecs);
  SBPL_FPRINTF(fout, "timetoturn45degsinplace(secs): %f\n", envIntervalLatCfg.timetoturn45degsinplace_secs);
  SBPL_FPRINTF(fout, "start(meters,rads): %f %f %f %f\n", DISCXY2CONT(envIntervalLatCfg.StartX_c, envIntervalLatCfg.cellsize_m), DISCXY2CONT(envIntervalLatCfg.StartY_c, envIntervalLatCfg.cellsize_m), DiscTheta2Cont(envIntervalLatCfg.StartTheta, ENVINTERVALLAT_THETADIRS), DISCTIME2CONT(envIntervalLatCfg.StartTime, envIntervalLatCfg.timeResolution));
  SBPL_FPRINTF(fout, "end(meters,rads): %f %f %f\n", DISCXY2CONT(envIntervalLatCfg.EndX_c, envIntervalLatCfg.cellsize_m), DISCXY2CONT(envIntervalLatCfg.EndY_c, envIntervalLatCfg.cellsize_m), DiscTheta2Cont(envIntervalLatCfg.EndTheta, ENVINTERVALLAT_THETADIRS));
  SBPL_FPRINTF(fout, "environment:\n");

  for(int y=0; y<envIntervalLatCfg.EnvHeight_c; y++){
    for(int x=0; x<envIntervalLatCfg.EnvWidth_c; x++){
      SBPL_FPRINTF(fout, "%d ", envIntervalLatCfg.Grid2D[x][y]);
    }
    SBPL_FPRINTF(fout, "\n");
  }
  fclose(fout);
}

void EnvIntervalLat::dumpDynamicObstaclesToFile(){
  FILE* fout = fopen("hall_dynobs.dob", "w");
  SBPL_FPRINTF(fout, "NumberOfDynamicObstacles: %d\n", (int)dynamicObstacles.size());
  for(unsigned int i=0; i<dynamicObstacles.size(); i++){
    SBPL_FPRINTF(fout, "DynamicObstacleID: %d\n", i);
    SBPL_FPRINTF(fout, "ObstacleRadius: %f\n", dynamicObstacles[i].radius - envIntervalLatCfg.robotRadius);
    SBPL_FPRINTF(fout, "NumberOfTrajectories: %d\n", (int)dynamicObstacles[i].trajectories.size());
    for(unsigned int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      SBPL_FPRINTF(fout, "TrajectoryID: %d\n", j);
      SBPL_FPRINTF(fout, "TrajectoryProbability: %f\n", dynamicObstacles[i].trajectories[j].prob);
      SBPL_FPRINTF(fout, "NumberOfPoints: %d\n", (int)dynamicObstacles[i].trajectories[j].points.size());
      for(unsigned int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_FPRINTF(fout, "%f %f %f %f\n", dynamicObstacles[i].trajectories[j].points[k].x, dynamicObstacles[i].trajectories[j].points[k].y, DISCTIME2CONT(dynamicObstacles[i].trajectories[j].points[k].t, envIntervalLatCfg.timeResolution), dynamicObstacles[i].trajectories[j].points[k].std_dev);
      }
      SBPL_FPRINTF(fout, "ObstacleExistsAfterTrajectory: %d\n", dynamicObstacles[i].trajectories[j].existsAfter);
    }
  }
  fclose(fout);
}

int EnvIntervalLat::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envIntervalLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	//get X, Y for the state
	envIntervalLatHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	envIntervalLatHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	
	//TODO - check if one of the gridsearches already computed and then use it.
	

	return (int)(ENVINTERVALLAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/envIntervalLatCfg.nominalvel_mpersecs);	

}


int EnvIntervalLat::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envIntervalLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	if(bNeedtoRecomputeGoalHeuristics)
	{
		grid2Dsearchfromgoal->search(envIntervalLatCfg.Grid2D, envIntervalLatCfg.cost_inscribed_thresh, 
			envIntervalLatCfg.EndX_c, envIntervalLatCfg.EndY_c, envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c,  
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeGoalHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c)
			/envIntervalLatCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
	int hEuclid = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, envIntervalLatCfg.EndX_c, envIntervalLatCfg.EndY_c));
		

#if DEBUG
	SBPL_FPRINTF(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/envIntervalLatCfg.nominalvel_mpersecs); 

}


int EnvIntervalLat::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in envIntervalLat... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif


	if(bNeedtoRecomputeStartHeuristics)
	{
		grid2Dsearchfromstart->search(envIntervalLatCfg.Grid2D, envIntervalLatCfg.cost_inscribed_thresh, 
			envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, envIntervalLatCfg.EndX_c, envIntervalLatCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeStartHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(envIntervalLatCfg.EndX_c, envIntervalLatCfg.EndY_c)
			/envIntervalLatCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(ENVINTERVALLAT_COSTMULT_MTOMM*EuclideanDistance_m(envIntervalLatCfg.StartX_c, envIntervalLatCfg.StartY_c, HashEntry->X, HashEntry->Y));
		

#if DEBUG
	SBPL_FPRINTF(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid); 
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/envIntervalLatCfg.nominalvel_mpersecs); 

}

int EnvIntervalLat::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}
//------------------------------------------------------------------------------
