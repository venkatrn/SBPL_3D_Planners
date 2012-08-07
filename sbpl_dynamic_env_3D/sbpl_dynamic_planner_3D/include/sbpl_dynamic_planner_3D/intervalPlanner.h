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
#ifndef __INTERVAL_PLANNER_H_
#define __INTERVAL_PLANNER_H_

#include <sbpl_dynamic_planner_3D/DiscreteSpaceTimeIntervalInformation.h>

//---configuration----

//control of EPS
#define IntervalPlanner_DEFAULT_INITIAL_EPS	    5.0
#define IntervalPlanner_DECREASE_EPS    1.0
//#define IntervalPlanner_FINAL_EPS	    1.0


//---------------------

#define IntervalPlanner_INCONS_LIST_ID 0

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;


//-------------------------------------------------------------

//state structure
typedef class INTERVALPLANNERSEARCHSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//IntervalPlanner* relevant data
	unsigned int v;
	unsigned int g;
	short unsigned int iterationclosed;
	short unsigned int callnumberaccessed;
	short unsigned int numofexpands;
	//best predecessor and the action from it, used only in forward searches
	CMDPSTATE *bestpredstate;
	//the next state if executing best action
	CMDPSTATE  *bestnextstate;
	unsigned int costtobestnextstate;
	int h;

  bool opt;
  unsigned int temp_g;
  CMDPSTATE* temp_bestpredstate;

	
public:
	INTERVALPLANNERSEARCHSTATEDATA() {};	
	~INTERVALPLANNERSEARCHSTATEDATA() {};
} IntervalPlannerState;

//-------------------------------------------------------------



//statespace
typedef struct INTERVALPLANNERSEARCHSTATESPACE
{
	double eps;
    double eps_satisfied;
	CHeap* heap;
	CList* inconslist;
	short unsigned int searchiteration;
	short unsigned int callnumber;
	CMDPSTATE* searchgoalstate;
	CMDPSTATE* searchstartstate;
	
	CMDP searchMDP;

	bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
	bool bNewSearchIteration;

} IntervalPlannerSearchStateSpace_t;



//IntervalPlanner* planner
class IntervalPlanner : public SBPLPlanner
{

public:
  /**
   * @brief Constructor
   * @param environment A pointer to the environment (already initialized)
   */
  IntervalPlanner(DiscreteSpaceTimeIntervalInformation* environment);

  /**
   * @brief Destructor
   */
  ~IntervalPlanner();

  /**
   * @brief Sets the start state
   * @param start_stateID The state ID of the start state (from the environment)
   * @return 1 on success, 0 otherwise
   */
  int set_start(int start_stateID);

  /**
   * @brief Sets the goal state
   * @param goal_stateID The state ID of the goal state (from the environment)
   * @return 1 on success, 0 otherwise
   */
  int set_goal(int goal_stateID);

  /**
   * @brief Runs the planner
   * @param allocated_time_secs The amount of time the planner can run (and try to improve the solution)
   * @param solution_stateIDs_V The path of state IDs from the start to goal (the environment's ConvertStateIDPathintoXYThetaPath can change a state ID path into a path of points)
   * @return 1 if a plan is found, 0 otherwise
   */
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);

  /**
   * @brief Runs the planner
   * @param allocated_time_secs The amount of time the planner can run (and try to improve the solution)
   * @param solution_stateIDs_V The path of state IDs from the start to goal (the environment's ConvertStateIDPathintoXYThetaPath can change a state ID path into a path of points)
   * @param solcost The cost of the solution
   * @return 1 if a plan is found, 0 otherwise
   */
	int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);

  /**
   * @brief Makes the planner start from scratch. This should be called before each replan.
   */
  int force_planning_from_scratch(); 

  /**
   * @brief This determines whether the planner improves the solution until time expires, or if it returns the first solution it finds.
   * @param bSearchUntilFirstSolution If this is true the planner will return the first solution found. (it also ignores the allocated time if this is true)
   */
	int set_search_mode(bool bSearchUntilFirstSolution);

  /**
   * @brief Returns the epsilon sub-optimality bound of the solution from the last replan
   */
	virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};

  /**
   * @brief Returns the number of expansions from the last replan
   */
  virtual int get_n_expands() const { return searchexpands; }

  /**
   * @brief Sets the final epsilon that the planner will work down to
   */
  void set_finalsolution_eps(double finalsolution_eps){IntervalPlanner_FINAL_EPS = finalsolution_eps;};

  /**
   * @brief Sets the initial epsilon that the planner starts at
   */
	virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};

  /**
   * @brief Sets the decrement step size for epsilon
   */
	virtual void set_decrease_eps_step(double dec_eps) {decrease_eps = dec_eps;};

  /**
   * @brief Returns all the search statistics from the last replan
   * @param solutionFound True if a solution was found
   * @param numExpands The number of expansions for each epsilon
   * @param solutionCost The solution cost for each epsilon
   * @param searchTime The planning time for each epsilon
   * @param searchEps The epsilons
   */
  void getSearchStats(bool* solutionFound, vector<int>* numExpands, vector<int>* solutionCost, vector<double>* searchTime, vector<double>* searchEps);

	void print_searchpath(FILE* fOut);
  void costs_changed(StateChangeQuery const & stateChange);
  void costs_changed();

private:
  int numOptStates;
  int numSubOptStates;
  int totalNumOptStates;
  int totalNumSubOptStates;

  DiscreteSpaceTimeIntervalInformation *environment_;
  double IntervalPlanner_FINAL_EPS;

	//member variables
	double finitial_eps;
  double decrease_eps;
	MDPConfig* MDPCfg_;

	bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

	bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    IntervalPlannerSearchStateSpace_t* pSearchStateSpace_;

	unsigned int searchexpands;
	int MaxMemoryCounter;
	clock_t TimeStarted;
	FILE *fDeb;
  bool solfound;
  double searchtime;
  int finalsolcost;

  vector<double> epsV;
  vector<unsigned int> expandsV;
  vector<unsigned int> solcostV;
  vector<double> searchTimeV;

	//member functions
	void Initialize_searchinfo(CMDPSTATE* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* CreateState(int stateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* GetState(int stateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	int ComputeHeuristic(CMDPSTATE* MDPstate, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//initialization of a state
	void InitializeSearchStateInfo(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//re-initialization of a state
	void ReInitializeSearchStateInfo(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	void DeleteSearchStateData(IntervalPlannerState* state);

	//used for backward search
	void UpdatePreds(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);


	//used for forward search
	void UpdateSuccs(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	int GetGVal(int StateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
	int ImprovePath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

	void BuildNewOPENList(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	void Reevaluatefvals(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//creates (allocates memory) search state space
	//does not initialize search statespace
	int CreateSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//deallocates memory used by SearchStateSpace
	void DeleteSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//debugging 
	void PrintSearchState(IntervalPlannerState* state, FILE* fOut);


	//reset properly search state space
	//needs to be done before deleting states
	int ResetSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//initialization before each search
	void ReInitializeSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//very first initialization
	int InitializeSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	int SetSearchGoalState(int SearchGoalStateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);


	int SetSearchStartState(int SearchStartStateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace);

	//reconstruct path functions are only relevant for forward search
	int ReconstructPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace);


	void PrintSearchPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

	int getHeurValue(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, int StateID);

	//get path 
	vector<int> GetSearchPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, int& solcost);


	bool Search(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);


};


#endif



