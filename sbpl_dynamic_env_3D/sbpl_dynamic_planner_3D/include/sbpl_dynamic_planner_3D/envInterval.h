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
#ifndef __ENVINTERVAL_H_
#define __ENVINTERVAL_H_

#include <sbpl_dynamic_planner_3D/DiscreteSpaceTimeIntervalInformation.h>
#include <sbpl_dynamic_planner_3D/sbpl_dynamicObstacles.h>

//eight-connected grid
#define ENVINTERVALLAT_DXYWIDTH 8

#define ENVINTERVALLAT_DEFAULTOBSTHRESH 254	//see explanation of the value below
#define ENVINTERVALLAT_DEFAULTDYNOBSTHRESH 254	//see explanation of the value below


//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define ENVINTERVALLAT_THETADIRS 16

//number of actions per x,y,theta state
#define ENVINTERVALLAT_DEFAULT_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define ENVINTERVALLAT_COSTMULT_MTOMM 1000

#define CONTTIME2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE)+0.5)):((int)((X)/(CELLSIZE))-1))
#define DISCTIME2CONT(X, CELLSIZE) ((X)*(CELLSIZE))

#define FAIL 0
#define SUCCESS_WITH_TIME 1
#define SUCCESS_NO_TIME 2

typedef struct
{
	char starttheta;
	char dX;
	char dY;
	char dZ;
	int dT;
  	char endtheta;
	unsigned int cost;
  
  //all cells within footprint of polygon for all pose on trajectory
	vector<SBPL_4Dcell_t> intersectingcellsV;
	
  //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
  //points along centerpoint trajectory (continuous)
	vector<SBPL_4Dpt_t> intermptV;
	
  //start at 0,0,starttheta and end at endcell in discrete domain
  //centerpoint cells along all poses in trajectory
	vector<SBPL_4Dcell_t> interm4DcellsV;
} envIntervalLatAction_t;


typedef struct 
{
	int stateID;
	int X;
	int Y;
	int Z;
	char Theta;
  int Interval;
  int T;
  int tempT;
  bool Optimal;
  bool closed;
  bool init;
	int iteration;
} envIntervalLatHashEntry_t;


typedef struct
{
	int motprimID;
	unsigned char starttheta_c;
	int additionalactioncostmult;
	SBPL_4Dcell_t endcell;
	//intermptV start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
	vector<SBPL_4Dpt_t> intermptV; 
}SBPL_xythetainterval_mprimitive;


//variables that dynamically change (e.g., array of states, ...)
typedef struct
{

	int startstateid;
	int goalstateid;

	bool bInitialized;

	//any additional variables


}EnvIntervalLat_t;

//configuration parameters
typedef struct envIntervalLat_config
{
	int EnvWidth_c;
	int EnvHeight_c;
	int EnvDepth_c;
	int StartX_c;
	int StartY_c;
	int StartZ_c;
	int StartTheta;
	int StartTime;
	int EndX_c;
	int EndY_c;
	int EndZ_c;
	int EndTheta;
	int numObs;
	unsigned char*** Grid3D;
        unsigned char** Grid2D;

	//the value at which and above which cells are obstacles in the maps sent from outside
	//the default is defined above
	unsigned char obsthresh; 

	//the value at which and above which until obsthresh (not including it) cells have the nearest obstacle at distance smaller than or equal to 
	//the inner circle of the robot. In other words, the robot is definitely colliding with the obstacle, independently of its orientation
	//if no such cost is known, then it should be set to obsthresh (if center of the robot collides with obstacle, then the whole robot collides with it
	//independently of its rotation)
	unsigned char cost_inscribed_thresh; 

	//the value at which and above which until cost_inscribed_thresh (not including it) cells 
	//**may** have a nearest osbtacle within the distance that is in between the robot inner circle and the robot outer circle
	//any cost below this value means that the robot will NOT collide with any obstacle, independently of its orientation
	//if no such cost is known, then it should be set to 0 or -1 (then no cell cost will lower than it, and therefore the robot's footprint will always be checked)
	int cost_possibly_circumscribed_thresh; //it has to be integer, because -1 means that it is not provided.

  //theshold for how close we get to a dynamic obstacle before it is called a collision
	unsigned char dynamic_obstacle_collision_cost_thresh;

	double nominalvel_mpersecs;
	double timetoturn45degsinplace_secs;
	double cellsize_m;
	double timeResolution;

	int dXY[ENVINTERVALLAT_DXYWIDTH][2];

	envIntervalLatAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
	vector<envIntervalLatAction_t*>* PredActionsV; //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i

	int actionwidth; //number of motion primitives
	vector<SBPL_xythetainterval_mprimitive> mprimV;

	vector<sbpl_3Dpt_t> FootprintPolygon;
  double robotRadius;
  double maxMovement;
} envIntervalLatConfig_t; 


//-------------------------------------------------------------------------//

class SBPL2DGridSearch;

class EnvIntervalLattice : public DiscreteSpaceTimeIntervalInformation
{

public:

  /**
   * @brief The constructor. One of the 3 initialize functions should be called after this.
   */
	EnvIntervalLattice();

  /**
   * @brief An initialize function if you have the costmap, motion primitive, and dynamic obstacles all in the proper text file formats.
   * @param sEnvFile The path to the costmap file
   * @param perimeterptsV A vector containing the (x,y,z) points of the polyhedral footprint assuming the robot was centered at (0,0,0). Leave empty to assume a circular robot (where the costmap is assumed to be inflated by the robot radius).
   * @param sMotPrimFile The path to the motion primitive file
   * @param sDynObsFile The path to the dynamic obstacle file
   * @return True if initialization was successful
   */
	bool InitializeEnv(const char* sEnvFile, const vector<sbpl_3Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sDynObsFile);	

  /**
   * @brief An initialize function if you have the costmap and motion primitive file, but would rather provide dynamic obstacle information using the setDynamicObstacles function.
   * @param sEnvFile The path to the costmap file
   * @param perimeterptsV A vector containing the (x,y,z) points of the polyhedral footprint assuming the robot was centered at (0,0,0). Leave empty to assume a circular robot (where the costmap is assumed to be inflated by the robot radius).
   * @param sMotPrimFile The path to the motion primitive file
   * @return True if initialization was successful
   */
	bool InitializeEnv(const char* sEnvFile, const vector<sbpl_3Dpt_t>& perimeterptsV, const char* sMotPrimFile);	

  /**
   * @brief An initialize function if you only have the motion primitive file. This is the initialize function generally used with a real simulator or robot. 
   * @param width Number of map cells along the x-axis
   * @param height Number of map cells along the y-axis
   * @param mapdata The initial costmap. If this is null, it is initialized to all freespace.
   * @param startx The initial start x in meters (this can be updated after init)
   * @param starty The initial start y in meters (this can be updated after init)
   * @param starttheta The initial start theta in radians (this can be updated after init)
   * @param starttime The initial start time in seconds (this can be updated after init)
   * @param goalx The initial goal x in meters (this can be updated after init)
   * @param goaly The initial goal y in meters (this can be updated after init)
   * @param goaltheta The initial goal theta in radians (this can be updated after init)
   * @param goaltol_x Goal x tolerance (not currently used)
   * @param goaltol_y Goal y tolerance (not currently used)
   * @param goaltol_theta Goal theta tolerance (not currently used)
   * @param perimeterptsV A vector containing the (x,y,z) points of the polyhedral footprint assuming the robot was centered at (0,0,0). Leave empty to assume a circular robot (where the costmap is assumed to be inflated by the robot radius).
   * @param cellsize_m The cell size in meters
   * @param timeResolution The time resolution in seconds (used when doing collision checking against dynamic obstacles)
   * @param temporal_padding_c The minimum amount of time between when an obstacle occupies a cell and when the robot can occupy that same cell (in seconds). This should be at least one timestep.
   * @param nominalvel_mpersecs The linear velocity of the robot (meters per second)
   * @param timetoturn45degsinplace_secs The amount of time it takes the robot to turn 45 degrees (seconds)
   * @param obsthresh The threshold value for obstacles. Costmap values greater or equal to this will be considered obstacles.
   * @param dynobsthresh A dynamic obstacle collision threshold (not used right now)
   * @param sMotPrimFile The path to the motion primitive file
   * @param dynObs A vector of the dynamic obstacles and their trajectories. (this can be updated after init)
   * @return True if initialization was successful
   */
  bool InitializeEnv(int width, int height, int depth,
         /** if mapdata is NULL the grid is initialized to all freespace */
           const unsigned char* mapdata,
           double startx, double starty, double startz, double starttheta, double startTime,
           double goalx, double goaly, double goalz, double goaltheta,
           double goaltol_x, double goaltol_y, double goaltol_z, double goaltol_theta,
           const vector<sbpl_3Dpt_t> & perimeterptsV,
           double cellsize_m, double timeResolution, double temporal_padding_c,
           double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
           unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile,
           vector<SBPL_DynamicObstacle_t> & dynObs);

  /**
   * @brief Initializes the 2D Grid used for heuristic computation
   * @return True if initialization is successful
   */
 bool InitializeGrid2D();

  /**
   * @brief Gets the state ID of the start state. This can be given to the planner when setting the planner start state.
   * @return start state ID
   */
  int getStartID();

  /**
   * @brief Gets the state ID of the goal state. This can be given to the planner when setting the planner goal state.
   * @return goal state ID
   */
  int getGoalID();

  /**
   * @brief Changes the cost of a cell in the map. This can be used to update the costmap from sensor updates between replans.
   * @param x The x coordinate of the cell to update
   * @param y The y coordinate of the cell to update
   * @param z The z coordinate of the cell to update
   * @param newcost The new value for the cell
   * @return True if the update was successful (right now it always returns true)
   */
  bool UpdateCost(int x, int y, int z, unsigned char newcost);

  /**
   * @brief Returns if the cell is an obstacle
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param z The z coordinate of the cell
   * @return True if the cell is an obstacle
   */
	bool IsObstacle(int x, int y, int z);

  /**
   * @brief Returns if the configuration is collision free on the static costmap
   * @param X The x coordinate of the center of the robot
   * @param Y The y coordinate of the center of the robot
   * @param Z The z coordinate of the center of the robot
   * @param Theta The orientation of the robot
   * @return True if the pose is safe
   */
	bool IsValidConfiguration(int X, int Y, int Z, int Theta);

  /**
   * @brief Returns the value of the cell
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param z The z coordinate of the cell
   * @return The value of the cell
   */
	unsigned char GetMapCost(int x, int y, int z);

  /**
   * @brief Returns if the coordinate is on the map
   * @param X The x coordinate
   * @param Y The y coordinate
   * @param Z The z coordinate
   * @return True if the coordinate is on the map
   */

  bool IsWithinMapCell(int X, int Y, int Z);

  /**
   * @brief Returns if the point is within the current footprint of the robot
   * @param pt The (x,y,z) coordinate of the point to be checked
   * @param bounding_polygon Pointer to a vector of coordinates specifying the current footprint of the robot 
   * @return True if the point is within the current footprint of the robot
   */

  bool IsInsideFootprint3D(sbpl_3Dpt_t pt, vector<sbpl_3Dpt_t>* bounding_polygon);

  /**
   * @brief Allows the setting of a few other parameters such as cost_inscribed_thresh and cost_possibly_circumscribed_thresh
   * @param parameter The name of the parameter
   * @param value The value to set it to
   * @return True if there were no errors
   */
	virtual bool SetEnvParameter(const char* parameter, int value);

  /**
   * @brief Get the value of parameters such as cost_inscribed_thresh and cost_possibly_circumscribed_thresh
   * @param parameter The name of the parameter
   * @return The value of the parameter
   */
	virtual int GetEnvParameter(const char* parameter);


  /**
   * @brief Get the value of parameters 
   */
	void GetEnvParms(int *size_x, int *size_y, int *size_z, double* startx, double* starty, double* startz, double* starttheta, double* startTime, double* goalx, double* goaly, double* goalz, double* goaltheta,
			double* cellsize_m, double* timeResolution, double* temporal_padding_, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, unsigned char* dyn_obs_thresh, vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV);

  /**
   * @brief Destructor
   */
  ~EnvIntervalLattice();

  
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
	virtual int  GetGoalHeuristic(int stateID) = 0;
	virtual int  GetStartHeuristic(int stateID) = 0;

	const envIntervalLatConfig_t* GetEnvNavConfig();
  void PrintTimeStat(FILE* fOut);
	void PrintEnv_Config(FILE* fOut);
  
  bool PoseContToDisc(double px, double py, double pz, double pth, double pt,
		      int &ix, int &iy, int &iz, int &ith, int &it) const;
  bool PoseDiscToCont(int ix, int iy, int iz, int ith, int it,
		      double &px, double &py, double &pz, double &pth, double &pt) const;
  virtual void PrintVars(){};

 protected:

  virtual int GetActionCost(int SourceX, int SourceY, int SourceZ, int SourceTheta, envIntervalLatAction_t* action);

  void InitializeTimelineMap();
  bool UpdateTimelineMap();
  void FillInBubble(int CenterX, int CenterY, int CenterZ, int T, int rad);
  void FillInBubbleSlice(int CenterX, int CenterY, int CenterZ, int T, int rad);
// TODO: Fill in z-slice

  void FillInBubbleColumn(int x, int topY, int botY, int z, int T);
  void FillInBubbleCell(int x, int y, int z, int T);

  void intervals2Timeline(int x, int y, int z);
  int getInterval(int x, int y, int z, int t);
  void getIntervals(vector<int>* intervals, vector<int>* times, envIntervalLatHashEntry_t* state, envIntervalLatAction_t* action);
  int getArrivalTimeToInterval(envIntervalLatHashEntry_t* state, envIntervalLatAction_t* action, int start_t, int end_t);
  virtual envIntervalLatHashEntry_t* getEntryFromID(int id) = 0;
  static bool pairCompare (pair<int,int> i, pair<int,int> j) { return (i.first<j.first); };

	//member data
	envIntervalLatConfig_t envIntervalLatCfg;
	EnvIntervalLat_t envIntervalLat;
	vector<SBPL_4Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
	vector<SBPL_4Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
	int iteration;
  vector<SBPL_DynamicObstacle_t> dynamicObstacles;
  vector< vector< vector< vector<int> > > > timelineMap;
  vector< vector< vector< vector< pair<int,int> > > > > intervalMap;
  int temporal_padding;

//TODO : Modify heuristic estimates to include z
	//2D search for heuristic computations
	bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
	bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
	SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
	SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells

  //load dynamic obstacles
  void ReadDynamicObstacles(FILE* fDynObs);

 	virtual void ReadConfiguration(FILE* fCfg);

	void InitializeEnvConfig(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV);


	bool CheckQuant(FILE* fOut);

	void SetConfiguration(int width, int height, int depth,
			      /** if mapdata is NULL the grid is initialized to all freespace */
			      const unsigned char* mapdata,
			      int startx, int starty, int startz, int starttheta, int startTime,
			      int goalx, int goaly, int goalz, int goaltheta,
				  double cellsize_m, double timeResolution,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_3Dpt_t> & robot_perimeterV);
	
	bool InitGeneral( vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV);
	void PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetainterval_mprimitive>* motionprimitiveV);
	void PrecomputeActions();

	void CreateStartandGoalStates();

	virtual void InitializeEnvironment() = 0;

	void ComputeHeuristicValues();

	bool IsValidCell(int X, int Y, int Z);

	void CalculateFootprintForPose(SBPL_4Dpt_t pose, vector<SBPL_4Dcell_t>* footprint);
	void RemoveSourceFootprint(SBPL_4Dpt_t sourcepose, vector<SBPL_4Dcell_t>* footprint);

  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch);
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch, vector<envIntervalLatAction_t*>* actionV=NULL) = 0;

	double EuclideanDistance_m(int X1, int Y1, int Z1, int X2, int Y2, int Z2);

	void ComputeReplanningData();
	void ComputeReplanningDataforAction(envIntervalLatAction_t* action);

	bool ReadMotionPrimitives(FILE* fMotPrims);
  bool ReadinMotionPrimitive(SBPL_xythetainterval_mprimitive* pMotPrim, FILE* fIn, bool &isWaitMotion);
	int ReadinCell(SBPL_4Dcell_t* cell, char* fIn);
	int ReadinPose(SBPL_4Dpt_t* pose, char* fIn);

	void PrintHeuristicValues();

  //Deprecated
	bool InitializeEnv(const char* sEnvFile);
	bool InitializeMDPCfg(MDPConfig *MDPCfg);
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
	virtual void SetAllPreds(CMDPSTATE* state);
	virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;
  bool InitializeEnv(int width, int height, int depth,
             const unsigned char* mapdata,
             double startx, double starty, double startz, double starttheta, double startTime,
             double goalx, double goaly, double goalz, double goaltheta,
					   double goaltol_x, double goaltol_y, double goaltol_z, double goaltol_theta,
					   const vector<sbpl_3Dpt_t> & perimeterptsV,
					   double cellsize_m, double timeResolution, double temporal_padding_c,
             double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
					   unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile);
	virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV) = 0;
	virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV) = 0;

};


class EnvIntervalLat : public EnvIntervalLattice
{

 public:
  ~EnvIntervalLat();

  /**
   * @brief Sets the start state of the environment. You still have to set the start state of the planner after this.
   * @param x The start x coordinate (meters)
   * @param y The start y coordinate (meters)
   * @param theta The start theta (radians)
   * @param startTime The start time (seconds)
   * @return The state ID of the start state
   */
  int SetStart(double x, double y, double z, double theta, double startTime);

  /**
   * @brief Sets the goal state of the environment. You still have to set the goal state of the planner after this.
   * @param x The goal x coordinate (meters)
   * @param y The goal y coordinate (meters)
   * @param theta The goal theta (radians)
   * @return The state ID of the goal state
   */
  int SetGoal(double x, double y, double z, double theta);

  /**
   * @brief Sets the dynamic obstacles
   * @param dynObs The dynamic obstacle vector
   * @param reset_states This resets the states between replans. This should always be true. This will reset even the start and goal states so make sure that SetStart and SetGoal are called after this.
   * @return True on success
   */
  bool setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states = true);
  

  /**
   * @brief Converts a path of state IDs (returned from the planner) into a paths of points (x,y,theta,time)
   * @param stateIDPath The path of state IDs
   * @param xythetaPath The path of points
   */
  void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath); 

  /**
   * @brief A function that gets the successors of a state. Used by the planner.
   */
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch, vector<envIntervalLatAction_t*>* actionV=NULL);

  /**
   * @brief A function that updates the time value for a state. Should only be called by the planner. 
   */
  void Relax(int stateID);

  /**
   * @brief Gets the coordinate given a state ID
   */
  void GetCoordFromState(int stateID, int& x, int& y, int& z, int& theta, int& t) const;
  
  /**
   * @brief Gets the coordinate (and whether it is optimal) given a state ID
   */
  void GetCoordFromState(int stateID, int& x, int& y, int& z, int& theta, int& t, bool& opt) const;

  /**
   * @brief Returns the state ID given the coordinate
   */
  int GetStateFromCoord(int x, int y, int z, int theta, int t);

  /**
   * @brief Get the timeline map 
   * @param parameter Reference to a variable whose type is the same as timelineMap
   * @return True if successful
   */
  bool GetTimelineMap(vector< vector< vector< vector<int> > > >& timelineMap);
  
  /**
   * @brief Returns the robot circumscribing radius, computed from the perimeter points. For a spherical robot, this is just the sphere radius
   * @return Robot radius
   */
  double GetRobotRadius();

  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

  virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
  virtual int  GetGoalHeuristic(int stateID);
  virtual int  GetStartHeuristic(int stateID);

  virtual int	 SizeofCreatedEnv();

  virtual void PrintVars(){};

  void getExpansions(vector<SBPL_4Dpt_t>* p);
  void dumpStatesToFile();
  void dumpEnvironmentToFile();
  void dumpDynamicObstaclesToFile();
 protected:
  void ClearStates();
  envIntervalLatHashEntry_t* getEntryFromID(int id);


  //hash table of size x_size*y_size. Maps from coords to stateId	
  int HashTableSize;
  vector<envIntervalLatHashEntry_t*>* Coord2StateIDHashTable;
  //vector that maps from stateID to coords	
  vector<envIntervalLatHashEntry_t*> StateID2CoordTable;

  unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z, unsigned int Theta, unsigned int interval, bool Opt);

  envIntervalLatHashEntry_t* GetHashEntry(int X, int Y, int Z, int Theta, int interval, int T, bool Opt);
  envIntervalLatHashEntry_t* CreateNewHashEntry(int X, int Y, int Z, int Theta, int interval, int T, bool Opt);

  virtual void InitializeEnvironment();

  void PrintHashTableHist();

  //Deprecated
  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
  void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
};


#endif

