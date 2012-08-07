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
#include <sbpl_dynamic_planner/intervalPlanner.h>


//-----------------------------------------------------------------------------------------------------

IntervalPlanner::IntervalPlanner(DiscreteSpaceTimeIntervalInformation* environment){//, bool bSearchForward){
  numOptStates = 0;
  numSubOptStates = 0;
  totalNumOptStates = 0;
  totalNumSubOptStates = 0;

  IntervalPlanner_FINAL_EPS = 1.0;
	bforwardsearch = true;//bSearchForward;

  environment_ = environment;
    
	bsearchuntilfirstsolution = false;
  finitial_eps = IntervalPlanner_DEFAULT_INITIAL_EPS;
  decrease_eps =  IntervalPlanner_DECREASE_EPS;
  searchexpands = 0;
  MaxMemoryCounter = 0;
  
  fDeb = SBPL_FOPEN("debug.txt", "w");
  
  pSearchStateSpace_ = new IntervalPlannerSearchStateSpace_t;
    
  //create the IntervalPlanner planner
  if(CreateSearchStateSpace(pSearchStateSpace_) != 1){
    SBPL_ERROR("ERROR: failed to create statespace\n");
    return;
  }

    //set the start and goal states
  if(InitializeSearchStateSpace(pSearchStateSpace_) != 1){
    SBPL_ERROR("ERROR: failed to create statespace\n");
    return;
  }    
}

IntervalPlanner::~IntervalPlanner(){
  if(pSearchStateSpace_ != NULL){
    //delete the statespace
    DeleteSearchStateSpace(pSearchStateSpace_);
    delete pSearchStateSpace_;
  }
  SBPL_FCLOSE(fDeb);
}


void IntervalPlanner::Initialize_searchinfo(CMDPSTATE* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	IntervalPlannerState* searchstateinfo = (IntervalPlannerState*)state->PlannerSpecificData;

	searchstateinfo->MDPstate = state;
	InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace); 
}


CMDPSTATE* IntervalPlanner::CreateState(int stateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){	
	CMDPSTATE* state = NULL;

#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] != -1){
		SBPL_ERROR("ERROR in CreateState: state already created\n");
		throw new SBPL_Exception();
	}
#endif

	//adds to the tail a state
	state = pSearchStateSpace->searchMDP.AddState(stateID);

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;

#if DEBUG
	if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]]){
		SBPL_ERROR("ERROR in CreateState: invalid state index\n");
		throw new SBPL_Exception();
	}
#endif

	//create search specific info
	state->PlannerSpecificData = (IntervalPlannerState*)malloc(sizeof(IntervalPlannerState));	
	Initialize_searchinfo(state, pSearchStateSpace);
	MaxMemoryCounter += sizeof(IntervalPlannerState);

	return state;
}

CMDPSTATE* IntervalPlanner::GetState(int stateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){	
	if(stateID >= (int)environment_->StateID2IndexMapping.size()){
    SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
		throw new SBPL_Exception();
	}

	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] == -1)
		return CreateState(stateID, pSearchStateSpace);
	else
		return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]];
}



//-----------------------------------------------------------------------------------------------------




int IntervalPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	//compute heuristic for search

	if(bforwardsearch){

#if MEM_CHECK == 1
    //int WasEn = DisableMemCheck();
#endif

  //forward search: heur = distance from state to searchgoal which is Goal IntervalPlannerState
    int retv =  environment_->GetGoalHeuristic(MDPstate->StateID);

#if MEM_CHECK == 1
    //if (WasEn)
    //	EnableMemCheck();
#endif

		return retv;
	}
	else{
		//backward search: heur = distance from searchgoal to state
		return environment_->GetStartHeuristic(MDPstate->StateID);
	}
}


//initialization of a state
void IntervalPlanner::InitializeSearchStateInfo(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	state->g = INFINITECOST;
	state->temp_g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[IntervalPlanner_INCONS_LIST_ID] = 0;
	state->numofexpands = 0;

	state->bestpredstate = NULL;
	state->temp_bestpredstate = NULL;

	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL)
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
	else 
		state->h = 0;
#else
	state->h = 0;
#endif
}



//re-initialization of a state
void IntervalPlanner::ReInitializeSearchStateInfo(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	state->g = INFINITECOST;
	state->temp_g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[IntervalPlanner_INCONS_LIST_ID] = 0;
	state->numofexpands = 0;

	state->bestpredstate = NULL;
	state->temp_bestpredstate = NULL;

	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL){
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
	}
	else 
		state->h = 0;
#else
	state->h = 0;
#endif


}



void IntervalPlanner::DeleteSearchStateData(IntervalPlannerState* state){
	//no memory was allocated
	MaxMemoryCounter = 0;
	return;
}



//used for backward search
void IntervalPlanner::UpdatePreds(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
  vector<int> PredIDV;
  vector<int> CostV;
	CKey key;
	IntervalPlannerState *predstate;

  environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

	//iterate through predecessors of s
	for(int pind = 0; pind < (int)PredIDV.size(); pind++){
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (IntervalPlannerState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace);

		//see if we can improve the value of predstate
		if(predstate->g > state->v + CostV[pind]){
			predstate->g = state->v + CostV[pind];
			predstate->bestnextstate = state->MDPstate;
			predstate->costtobestnextstate = CostV[pind];

			//re-insert into heap if not closed yet
			if(predstate->iterationclosed != pSearchStateSpace->searchiteration){
				key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
				//key.key[1] = predstate->h;
				if(predstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(predstate,key);
				else
					pSearchStateSpace->heap->insertheap(predstate,key);
			}
			//take care of incons list
			else if(predstate->listelem[IntervalPlanner_INCONS_LIST_ID] == NULL){
				pSearchStateSpace->inconslist->insert(predstate, IntervalPlanner_INCONS_LIST_ID);
			}
		}
	} //for predecessors
}

//used for forward search
void IntervalPlanner::UpdateSuccs(IntervalPlannerState* state, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<bool> OptV;
	CKey key;
	IntervalPlannerState *succstate;

  if(state->opt)
    numOptStates++;
  else
    numSubOptStates++;

  environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV, &OptV, pSearchStateSpace->eps==1.0);

	//iterate through successors of s
	for(int sind = 0; sind < (int)SuccIDV.size(); sind++){
		CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
		int cost = CostV[sind];

		succstate = (IntervalPlannerState*)(SuccMDPState->PlannerSpecificData);
		if(succstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(succstate, pSearchStateSpace);

		//see if we can improve the value of succstate
		//taking into account the cost of action

		if(succstate->temp_g > state->v + cost){
      succstate->temp_g = state->v + cost;
      succstate->temp_bestpredstate = state->MDPstate; 

			//re-insert into heap if not closed yet
			if(succstate->iterationclosed != pSearchStateSpace->searchiteration){

        succstate->g = state->v + cost;
        succstate->bestpredstate = state->MDPstate; 

        succstate->opt = OptV[sind];

        environment_->Relax(succstate->MDPstate->StateID);

        if(OptV[sind])
          key.key[0] = (int)(pSearchStateSpace->eps*(succstate->g + succstate->h));
        else
          key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);

				//key.key[1] = succstate->h;

				if(succstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(succstate,key);
				else
          pSearchStateSpace->heap->insertheap(succstate,key);
			}
			//take care of incons list
			else if(succstate->listelem[IntervalPlanner_INCONS_LIST_ID] == NULL)
				pSearchStateSpace->inconslist->insert(succstate, IntervalPlanner_INCONS_LIST_ID);
		} 

	} //for actions
}

//TODO-debugmax - add obsthresh and other thresholds to other environments in 3dkin
int IntervalPlanner::GetGVal(int StateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	 CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
	 IntervalPlannerState* state = (IntervalPlannerState*)cmdp_state->PlannerSpecificData;
	 return state->g;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int IntervalPlanner::ImprovePath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs){
	int expands;
	IntervalPlannerState *state, *searchgoalstate;
	CKey key, minkey;
	CKey goalkey;

	expands = 0;


	if(pSearchStateSpace->searchgoalstate == NULL){
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}

	//goal state
	searchgoalstate = (IntervalPlannerState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);

	//set goal key
	goalkey.key[0] = searchgoalstate->g;
	//goalkey.key[1] = searchgoalstate->h;

	//expand states until done
	minkey = pSearchStateSpace->heap->getminkeyheap();
	CKey oldkey = minkey;
	while(!pSearchStateSpace->heap->emptyheap() && minkey.key[0] < INFINITECOST && goalkey > minkey &&
		(clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC){

		//get the state		
		state = (IntervalPlannerState*)pSearchStateSpace->heap->deleteminheap();


#if DEBUG
		//SBPL_FPRINTF(fDeb, "expanding state(%d): h=%d g=%u key=%u v=%u iterc=%d callnuma=%d expands=%d (g(goal)=%u)\n",
		//	state->MDPstate->StateID, state->h, state->g, state->g+(int)(pSearchStateSpace->eps*state->h), state->v, 
		//	state->iterationclosed, state->callnumberaccessed, state->numofexpands, searchgoalstate->g);
		SBPL_FPRINTF(fDeb, "expanding: ");
		PrintSearchState(state, fDeb);
		if(state->listelem[IntervalPlanner_INCONS_LIST_ID]  != NULL){
			SBPL_FPRINTF(fDeb, "ERROR: expanding a state from inconslist\n");
			SBPL_ERROR("ERROR: expanding a state from inconslist\n");
			throw new SBPL_Exception();
		}
		fflush(fDeb);
#endif

#if DEBUG
		if(minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS){
			//SBPL_PRINTF("WARN in search: the sequence of keys decreases\n");
			//throw new SBPL_Exception();
		}
		oldkey = minkey;
#endif

		if(state->v == state->temp_g){
			SBPL_ERROR("ERROR: consistent state is being expanded (v=%d, temp_g=%d g=%d)\n",state->v,state->temp_g,state->g);
#if DEBUG
			SBPL_FPRINTF(fDeb, "ERROR: consistent state is being expanded\n");
			throw new SBPL_Exception();
#endif
		}

		//recompute state value      
		state->v = state->g;
    state->temp_g = state->v; //This is just so that the debugging comparison above is more useful
		state->iterationclosed = pSearchStateSpace->searchiteration;

		//new expand      
		expands++;
		state->numofexpands++;


		if(bforwardsearch == false)
			UpdatePreds(state, pSearchStateSpace);
		else
			UpdateSuccs(state, pSearchStateSpace);
		
		//recompute minkey
		minkey = pSearchStateSpace->heap->getminkeyheap();

		//recompute goalkey if necessary
		if(goalkey.key[0] != (int)searchgoalstate->g){
			//SBPL_PRINTF("re-computing goal key\n");
			//recompute the goal key (heuristics should be zero)
			goalkey.key[0] = searchgoalstate->g;
			//goalkey.key[1] = searchgoalstate->h;
		}

		if(expands%100000 == 0 && expands > 0){
			SBPL_PRINTF("expands so far=%u\n", expands);
		}

	}

	int retv = 1;
	if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap()){
		SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
		retv = 0;
	}
	else if(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey){
		SBPL_PRINTF("search exited because it ran out of time\n");
		retv = 2;
	}
	else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap()){
		SBPL_PRINTF("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
		retv = 0;
	}
	else{
		SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
		retv = 1;
	}

	//SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);

	searchexpands += expands;

	return retv;		
}


void IntervalPlanner::BuildNewOPENList(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	IntervalPlannerState *state;
	CKey key;
	CHeap* pheap = pSearchStateSpace->heap;
	CList* pinconslist = pSearchStateSpace->inconslist; 
		
	//move incons into open
	while(pinconslist->firstelement != NULL){
    state = (IntervalPlannerState*)pinconslist->firstelement->liststate;

    state->g = state->temp_g;
    state->bestpredstate = state->temp_bestpredstate;
    environment_->Relax(state->MDPstate->StateID);
    
    //compute f-value
    if(state->opt)
      key.key[0] = (int)(pSearchStateSpace->eps*(state->g + state->h));
    else
      key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
    //key.key[1] = state->h;
    
    //insert into OPEN
    pheap->insertheap(state, key);
    //remove from INCONS
    pinconslist->remove(state, IntervalPlanner_INCONS_LIST_ID);
  }
}


void IntervalPlanner::Reevaluatefvals(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	CKey key;
	int i;
	CHeap* pheap = pSearchStateSpace->heap;
	
	//recompute priorities for states in OPEN and reorder it
	for (i = 1; i <= pheap->currentsize; ++i){
		IntervalPlannerState* state = (IntervalPlannerState*)pheap->heap[i].heapstate;
    if(state->opt)
      pheap->heap[i].key.key[0] = (int)(pSearchStateSpace->eps*(state->g + state->h)); 
    else
      pheap->heap[i].key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h); 
  }
	pheap->makeheap();

	pSearchStateSpace->bReevaluatefvals = false;
}




//creates (allocates memory) search state space
//does not initialize search statespace
int IntervalPlanner::CreateSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	//create a heap
	pSearchStateSpace->heap = new CHeap;
	pSearchStateSpace->inconslist = new CList;
	MaxMemoryCounter += sizeof(CHeap);
	MaxMemoryCounter += sizeof(CList);

	pSearchStateSpace->searchgoalstate = NULL;
	pSearchStateSpace->searchstartstate = NULL;

	searchexpands = 0;


  pSearchStateSpace->bReinitializeSearchStateSpace = false;
	
	return 1;
}

//deallocates memory used by SearchStateSpace
void IntervalPlanner::DeleteSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	if(pSearchStateSpace->heap != NULL){
		pSearchStateSpace->heap->makeemptyheap();
		delete pSearchStateSpace->heap;
		pSearchStateSpace->heap = NULL;
	}

	if(pSearchStateSpace->inconslist != NULL){
		pSearchStateSpace->inconslist->makeemptylist(IntervalPlanner_INCONS_LIST_ID);
		delete pSearchStateSpace->inconslist;
		pSearchStateSpace->inconslist = NULL;
	}

	//delete the states themselves
	int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
	for(int i=0; i < iend; i++){
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
    if(state != NULL && state->PlannerSpecificData != NULL){
      DeleteSearchStateData((IntervalPlannerState*)state->PlannerSpecificData);
      free((IntervalPlannerState*)state->PlannerSpecificData);
      state->PlannerSpecificData = NULL;
    }
	}
	pSearchStateSpace->searchMDP.Delete();
}



//reset properly search state space
//needs to be done before deleting states
int IntervalPlanner::ResetSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(IntervalPlanner_INCONS_LIST_ID);

	return 1;
}

//initialization before each search
void IntervalPlanner::ReInitializeSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	CKey key;

	//increase callnumber
	pSearchStateSpace->callnumber++;

	//reset iteration
	pSearchStateSpace->searchiteration = 0;
	pSearchStateSpace->bNewSearchIteration = true;

#if DEBUG
  SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n", 
          pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif

	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(IntervalPlanner_INCONS_LIST_ID);

    //reset 
	pSearchStateSpace->eps = this->finitial_eps;
  pSearchStateSpace->eps_satisfied = INFINITECOST;

	//initialize start state
	IntervalPlannerState* startstateinfo = (IntervalPlannerState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
	if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

	startstateinfo->g = 0;
	startstateinfo->temp_g = 0;
  startstateinfo->opt = true;

	//insert start state into the heap
	key.key[0] = (long int)(pSearchStateSpace->eps*startstateinfo->h);
	//key.key[1] = startstateinfo->h;
	pSearchStateSpace->heap->insertheap(startstateinfo, key);

  pSearchStateSpace->bReinitializeSearchStateSpace = false;
	pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int IntervalPlanner::InitializeSearchStateSpace(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	if(pSearchStateSpace->heap->currentsize != 0 || 
		 pSearchStateSpace->inconslist->currentsize != 0){
		SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
		throw new SBPL_Exception();
	}

	pSearchStateSpace->eps = this->finitial_eps;
  pSearchStateSpace->eps_satisfied = INFINITECOST;
	pSearchStateSpace->searchiteration = 0;
	pSearchStateSpace->bNewSearchIteration = true;
	pSearchStateSpace->callnumber = 0;
	pSearchStateSpace->bReevaluatefvals = false;

	//create and set the search start state
	pSearchStateSpace->searchgoalstate = NULL;
	//pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
  pSearchStateSpace->searchstartstate = NULL;
	
  pSearchStateSpace->bReinitializeSearchStateSpace = true;

	return 1;
}


int IntervalPlanner::SetSearchGoalState(int SearchGoalStateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	if(pSearchStateSpace->searchgoalstate == NULL || 
		 pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID){
		pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

		//should be new search iteration
		pSearchStateSpace->eps_satisfied = INFINITECOST;
		pSearchStateSpace->bNewSearchIteration = true;
		pSearchStateSpace_->eps = this->finitial_eps;


		//recompute heuristic for the heap if heuristics is used
#if USE_HEUR
		for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++){
			CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
			IntervalPlannerState* state = (IntervalPlannerState*)MDPstate->PlannerSpecificData;
			state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
		}
		
		pSearchStateSpace->bReevaluatefvals = true;
#endif
  }

	return 1;
}


int IntervalPlanner::SetSearchStartState(int SearchStartStateID, IntervalPlannerSearchStateSpace_t* pSearchStateSpace){
	CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);

	if(MDPstate !=  pSearchStateSpace->searchstartstate){	
		pSearchStateSpace->searchstartstate = MDPstate;
		pSearchStateSpace->bReinitializeSearchStateSpace = true;
	}

	return 1;
}



int IntervalPlanner::ReconstructPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace){	
	if(bforwardsearch){ //nothing to do, if search is backward
		CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
		CMDPSTATE* PredMDPstate;
		IntervalPlannerState *predstateinfo, *stateinfo;



#if DEBUG
		SBPL_FPRINTF(fDeb, "reconstructing a path:\n");
#endif

		while(MDPstate != pSearchStateSpace->searchstartstate){
			stateinfo = (IntervalPlannerState*)MDPstate->PlannerSpecificData;

#if DEBUG
			PrintSearchState(stateinfo, fDeb);
#endif
			if(stateinfo->g == INFINITECOST){	
				//SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
				//throw new SBPL_Exception();
				return -1;
			}

			if(stateinfo->bestpredstate == NULL){
				SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
				throw new SBPL_Exception();
			}

			//get the parent state
			PredMDPstate = stateinfo->bestpredstate;
			predstateinfo = (IntervalPlannerState*)PredMDPstate->PlannerSpecificData;

			//set its best next info
			predstateinfo->bestnextstate = MDPstate;

			//check the decrease of g-values along the path
			if(predstateinfo->v >= stateinfo->g){
				SBPL_ERROR("ERROR in ReconstructPath: g-values are non-decreasing\n");			
				PrintSearchState(predstateinfo, fDeb);
				throw new SBPL_Exception();
			}

			//transition back
			MDPstate = PredMDPstate;
		}
	}

	return 1;
}



void IntervalPlanner::PrintSearchPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, FILE* fOut){
	IntervalPlannerState* searchstateinfo;
	CMDPSTATE* state;
	int goalID;
	int PathCost;

	if(bforwardsearch){
		state  = pSearchStateSpace->searchstartstate;
		goalID = pSearchStateSpace->searchgoalstate->StateID;
	}
	else{
		state = pSearchStateSpace->searchgoalstate;
		goalID = pSearchStateSpace->searchstartstate->StateID;
	}
	if(fOut == NULL)
		fOut = stdout;

	PathCost = ((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

	SBPL_FPRINTF(fOut, "Printing a path from state %d to the goal state %d\n", 
          state->StateID, pSearchStateSpace->searchgoalstate->StateID);
	SBPL_FPRINTF(fOut, "Path cost = %d:\n", PathCost);
			
	environment_->PrintState(state->StateID, false, fOut);

	int costFromStart = 0;
	while(state->StateID != goalID){
		SBPL_FPRINTF(fOut, "state %d ", state->StateID);

		if(state->PlannerSpecificData == NULL){
			SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
			break;
		}

		searchstateinfo = (IntervalPlannerState*)state->PlannerSpecificData;

		if(searchstateinfo->bestnextstate == NULL){
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}
		if(searchstateinfo->g == INFINITECOST){
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

		int costToGoal = PathCost - costFromStart;
		int transcost = searchstateinfo->g - ((IntervalPlannerState*)(searchstateinfo->bestnextstate->PlannerSpecificData))->v;
		if(bforwardsearch)
			transcost = -transcost;

		costFromStart += transcost;

		SBPL_FPRINTF(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g, 			
            searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

		state = searchstateinfo->bestnextstate;

		environment_->PrintState(state->StateID, false, fOut);
	}
}

void IntervalPlanner::PrintSearchState(IntervalPlannerState* state, FILE* fOut){
	SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d expands=%d heapind=%d inconslist=%d\n",
          state->MDPstate->StateID, state->h, state->g, state->v, 
          state->iterationclosed, state->callnumberaccessed, state->numofexpands, state->heapindex, state->listelem[IntervalPlanner_INCONS_LIST_ID]?1:0);
	environment_->PrintState(state->MDPstate->StateID, true, fOut);
}



int IntervalPlanner::getHeurValue(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, int StateID){
	CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
	IntervalPlannerState* searchstateinfo = (IntervalPlannerState*)MDPstate->PlannerSpecificData;
	return searchstateinfo->h;
}


vector<int> IntervalPlanner::GetSearchPath(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, int& solcost){
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<bool> OptV;
  vector<int> wholePathIds;
  IntervalPlannerState* searchstateinfo;
  CMDPSTATE* state = NULL; 
  CMDPSTATE* goalstate = NULL;
  CMDPSTATE* startstate=NULL;
  
  if(bforwardsearch){	
    startstate = pSearchStateSpace->searchstartstate;
    goalstate = pSearchStateSpace->searchgoalstate;
    
    //reconstruct the path by setting bestnextstate pointers appropriately
    ReconstructPath(pSearchStateSpace);
  }
  else{
    startstate = pSearchStateSpace->searchgoalstate;
    goalstate = pSearchStateSpace->searchstartstate;
  }
  
  state = startstate;
  
  wholePathIds.push_back(state->StateID);
  solcost = 0;
  
  FILE* fOut = stdout;
  while(state->StateID != goalstate->StateID){
    if(state->PlannerSpecificData == NULL){
      SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
      break;
    }
      
    searchstateinfo = (IntervalPlannerState*)state->PlannerSpecificData;
    
    if(searchstateinfo->bestnextstate == NULL){
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }
    if(searchstateinfo->g == INFINITECOST){
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }
      
    environment_->GetSuccs(state->StateID, &SuccIDV, &CostV, &OptV, false/*pSearchStateSpace->eps==1.0*/);
    int actioncost = INFINITECOST;
    for(int i = 0; i < (int)SuccIDV.size(); i++){   
      if(SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID && CostV.at(i)<actioncost)
        actioncost = CostV.at(i);
    }
    if(actioncost == INFINITECOST)
      SBPL_PRINTF("WARNING: actioncost = %d (%d)\n", actioncost,state->StateID);
      
    solcost += actioncost;
      
    //SBPL_FPRINTF(fDeb, "actioncost=%d between states %d and %d\n", 
    //        actioncost, state->StateID, searchstateinfo->bestnextstate->StateID);
    //environment_->PrintState(state->StateID, false, fDeb);
    //environment_->PrintState(searchstateinfo->bestnextstate->StateID, false, fDeb);
      
      
#if DEBUG
    IntervalPlannerState* nextstateinfo = (IntervalPlannerState*)(searchstateinfo->bestnextstate->PlannerSpecificData);
    if(actioncost != abs((int)(searchstateinfo->g - nextstateinfo->g)) && pSearchStateSpace->eps_satisfied <= 1.001){
      SBPL_FPRINTF(fDeb, "ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
              actioncost, abs((int)(searchstateinfo->g - nextstateinfo->g)));
      SBPL_ERROR("ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
             actioncost,abs((int)(searchstateinfo->g - nextstateinfo->g)));
      PrintSearchState(searchstateinfo, fDeb);
      PrintSearchState(nextstateinfo, fDeb);
    }
#endif
      
    state = searchstateinfo->bestnextstate;
      
    wholePathIds.push_back(state->StateID);
  }

  return wholePathIds;
}



bool IntervalPlanner::Search(IntervalPlannerSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs){
	CKey key;
	TimeStarted = clock();
  searchexpands = 0;

#if DEBUG
	SBPL_FPRINTF(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

  if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
    //re-initialize state space 
    ReInitializeSearchStateSpace(pSearchStateSpace);
  }

	if(bOptimalSolution){
		pSearchStateSpace->eps = 1;
		MaxNumofSecs = INFINITECOST;
	}
	else if(bFirstSolution){
		MaxNumofSecs = INFINITECOST;
	}

  epsV.clear();
  expandsV.clear(); 
  solcostV.clear();
  searchTimeV.clear();

	//the main loop of IntervalPlanner*
	int prevexpands = 0;
	clock_t loop_time;
	while(pSearchStateSpace->eps_satisfied > IntervalPlanner_FINAL_EPS && 
		   (clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC){
    loop_time = clock();
		//decrease eps for all subsequent iterations
		if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution){
			pSearchStateSpace->eps = pSearchStateSpace->eps - decrease_eps;
			if(pSearchStateSpace->eps < IntervalPlanner_FINAL_EPS)
				pSearchStateSpace->eps = IntervalPlanner_FINAL_EPS;

			//the priorities need to be updated
			pSearchStateSpace->bReevaluatefvals = true; 

			//it will be a new search
			pSearchStateSpace->bNewSearchIteration = true;

			//build a new open list by merging it with incons one
			BuildNewOPENList(pSearchStateSpace); 
		}

		if(pSearchStateSpace->bNewSearchIteration){
			pSearchStateSpace->searchiteration++;
			pSearchStateSpace->bNewSearchIteration = false;
		}

		//re-compute f-values if necessary and reorder the heap
		if(pSearchStateSpace->bReevaluatefvals) 
			Reevaluatefvals(pSearchStateSpace);

		//improve or compute path
		if(ImprovePath(pSearchStateSpace, MaxNumofSecs) == 1){
      pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps;
      epsV.push_back(pSearchStateSpace->eps);
      expandsV.push_back(searchexpands-prevexpands);
      solcostV.push_back(((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g);
      searchTimeV.push_back(double(clock()-loop_time)/CLOCKS_PER_SEC);
    }

		//print the solution cost and eps bound
		SBPL_PRINTF("eps=%f expands=%d(opt=%d, sub=%d) g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,numOptStates,numSubOptStates,
           ((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
    totalNumOptStates += numOptStates;
    totalNumSubOptStates += numSubOptStates;
    numOptStates = 0;
    numSubOptStates = 0;

#if DEBUG
    SBPL_FPRINTF(fDeb, "eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
          ((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
		PrintSearchState((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
		prevexpands = searchexpands;

		//if just the first solution then we are done
		if(bFirstSolution)
			break;

		//no solution exists
		if(((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
			break;
	}

#if DEBUG
	fflush(fDeb);
#endif

	PathCost = ((IntervalPlannerState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
	MaxMemoryCounter += environment_->StateID2IndexMapping.size()*sizeof(int);
	
	SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter);

	int solcost = INFINITECOST;
	bool ret = false;
	if(PathCost == INFINITECOST){
		SBPL_PRINTF("could not find a solution\n");
		ret = false;
	}
	else{
		SBPL_PRINTF("solution is found\n");      
    pathIds = GetSearchPath(pSearchStateSpace, solcost);
    ret = true;
	}

  solfound = ret;
  searchtime = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
  finalsolcost = solcost;
	SBPL_PRINTF("total expands this call = %d(opt=%d, sub=%d), planning time = %.3f secs, solution cost=%d\n", 
           searchexpands, totalNumOptStates, totalNumSubOptStates, searchtime, solcost);
  totalNumOptStates = 0;
  totalNumSubOptStates = 0;
    
  //SBPL_FPRINTF(fStat, "%d %d\n", searchexpands, solcost);

	return ret;
}


//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int IntervalPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V){
	int solcost;
	return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

//returns 1 if found a solution, and 0 otherwise
int IntervalPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost){
  vector<int> pathIds; 
  bool bFound = false;
  int PathCost;
  bool bFirstSolution = this->bsearchuntilfirstsolution;
  bool bOptimalSolution = false;
  *psolcost = 0;
  
  SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);
  
  //plan
  if((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false){
    SBPL_PRINTF("failed to find a solution\n");
  }
  
  //copy the solution
  *solution_stateIDs_V = pathIds;
  *psolcost = PathCost;

  //environment_->PostProcess(solution_stateIDs_V, psolcost);
  
  return (int)bFound;
}


int IntervalPlanner::set_goal(int goal_stateID){
	SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
	environment_->PrintState(goal_stateID, true, stdout);

	if(bforwardsearch){	
		if(SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1){
      SBPL_ERROR("ERROR: failed to set search goal state\n");
      return 0;
    }
	}
	else{
    if(SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1){
      SBPL_ERROR("ERROR: failed to set search start state\n");
      return 0;
    }
	}
  return 1;
}


int IntervalPlanner::set_start(int start_stateID){
	SBPL_PRINTF("planner: setting start to %d\n", start_stateID);
	environment_->PrintState(start_stateID, true, stdout);

	if(bforwardsearch){	
    if(SetSearchStartState(start_stateID, pSearchStateSpace_) != 1){
      SBPL_ERROR("ERROR: failed to set search start state\n");
      return 0;
    }
	}
	else{
    if(SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1){
      SBPL_ERROR("ERROR: failed to set search goal state\n");
      return 0;
    }
	}
  return 1;
}



void IntervalPlanner::costs_changed(StateChangeQuery const & stateChange){
  pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}

void IntervalPlanner::costs_changed(){
  pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}



int IntervalPlanner::force_planning_from_scratch(){
	SBPL_PRINTF("planner: forceplanfromscratch set\n");
  pSearchStateSpace_->bReinitializeSearchStateSpace = true;
  return 1;
}


int IntervalPlanner::set_search_mode(bool bSearchUntilFirstSolution){
	SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);
	bsearchuntilfirstsolution = bSearchUntilFirstSolution;
	return 1;
}


void IntervalPlanner::print_searchpath(FILE* fOut){
	PrintSearchPath(pSearchStateSpace_, fOut);
}


//---------------------------------------------------------------------------------------------------------

//function for getting search data
void IntervalPlanner::getSearchStats(bool* solutionFound, vector<int>* numExpands, vector<int>* solutionCost, vector<double>* searchTime, vector<double>* searchEps){
  *solutionFound = solfound;
  for(int i=0; i<epsV.size(); i++){
    searchEps->push_back(epsV[i]);
    numExpands->push_back(expandsV[i]);
    solutionCost->push_back(solcostV[i]);
    searchTime->push_back(searchTimeV[i]);
  }
}

