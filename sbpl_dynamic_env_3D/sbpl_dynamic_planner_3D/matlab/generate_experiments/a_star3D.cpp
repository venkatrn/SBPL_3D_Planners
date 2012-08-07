#include <math.h>
#include <vector>
#include <set>
#include "mex.h"
#include "time.h"

using namespace std;

#define max(a,b) (a>=b ? a : b)
#define min(a,b) (a<=b ? a : b)

#define TIME_DEBUG 1

typedef struct{
    int x;
    int y;
    int z;
    double g;
    double h;
    bool closed;
} state_t;

vector<state_t> states;

int startX;
int startY;
int startZ;

int transZ[10] = {0,0,0,0,0,0,0,0,1,-1};
int transY[10] = {-1,-1,-1,0,0,1,1,1,0,0};
int transX[10] = {-1,0,1,-1,1,-1,0,1,0,0};
double transC[10] = {sqrt(2),1,sqrt(2),1,1,sqrt(2),1,sqrt(2),1,1};


class stateComparison{
public:
  //stateComparison(){}
  bool operator() (const state_t* s1, const state_t* s2) const{
      //mexPrintf("%x %x\n",s1,s2);
      if(s1==s2)
          return false;
      double f1 = s1->g+s1->h;
      double f2 = s2->g+s2->h;
      if(f1==f2)
          return s1 < s2;
      return f1 < f2;
  }
};


double getH(int x, int y, int z){
    return 0;//sqrt((x-startX)*(x-startX) + (y-startY)*(y-startY));
}

void getPreds(state_t* state, vector<state_t*>& preds, vector<double>& costs, double* map, int mapY, int mapX, int mapZ){
    int i;
    
    //if(state->x == 153 && state->y == 221)
    //mexPrintf("s: x=%d y=%d c=%f\n",state->x+1,state->y+1,state->g);
    for(i=0; i<10; i++){
        int newX = state->x + transX[i];
        int newY = state->y + transY[i];
	int newZ = state->z + transZ[i];
        
        //skip this pred if we are off the map
        if(newX < 0 || newX >= mapX || newY < 0 || newY >= mapY || newZ < 0 || newZ >= mapZ)
            continue;
	if(map[newY+mapY*newX+mapY*mapX*newZ])
	    continue;
        
        double c = transC[i];
        
        //if(state->x == 153 && state->y == 221)
        //mexPrintf("    s: x=%d y=%d c=%f\n",newX+1,newY+1,state->g+c);
        preds.push_back(&states[newZ*mapY*mapX + newY*mapX + newX]);
        costs.push_back(c);
    }
}

void runAStar(double* map, int mapY, int mapX, int mapZ, double* start, double* goal, vector<double>& pathX, vector<double>& pathY, vector<double>& pathZ){
    states.resize(mapY*mapX*mapZ);
    startX = ((int)(start[0]+0.5))-1;
    startY = ((int)(start[1]+0.5))-1;
    startZ = ((int)(start[2]+0.5))-1;
    int goalX = ((int)(goal[0]+0.5))-1;
    int goalY = ((int)(goal[1]+0.5))-1;
    int goalZ = ((int)(goal[2]+0.5))-1;
    mexPrintf("start(%d,%d,%d) goal(%d,%d,%d)\n",startX,startY,startZ,goalX,goalY,goalZ);
    
    mexPrintf("map size (%d %d %d)\n",mapX,mapY,mapZ);
    
    #if TIME_DEBUG
    clock_t t0 = clock();
    #endif
   
  for(int z=0; z<mapZ; z++){
    for(int y=0; y<mapY; y++){
        for(int x=0; x<mapX; x++){
            states[z*mapY*mapX+y*mapX+x].x = x;
            states[z*mapY*mapX+y*mapX+x].y = y;
	    states[z*mapY*mapX+y*mapX+x].z = z;
            states[z*mapY*mapX+y*mapX+x].g = -1;
            states[z*mapY*mapX+y*mapX+x].h = getH(x,y,z);
            states[z*mapY*mapX+y*mapX+x].closed = false;
            //mexPrintf("%f ",map[y+mapY*x]);
        }
        //mexPrintf("\n",);
    }
  }
    states[goalZ*mapY*mapX+goalY*mapX+goalX].g = 0;
    mexPrintf("goal state %d %d %d\n",states[goalZ*mapY*mapX+goalY*mapX+goalX].x,states[goalZ*mapY*mapX+goalY*mapX+goalX].y,states[goalZ*mapY*mapX+goalY*mapX+goalX].z);
    set<state_t*, stateComparison> queue;
    queue.insert(&states[goalZ*mapY*mapX+goalY*mapX+goalX]);
    
    #if TIME_DEBUG
    clock_t t1 = clock();
    mexPrintf("init time = %f\n", ((double)(t1-t0))/CLOCKS_PER_SEC);
    t0 = clock();
    #endif
    
    while(!queue.empty()){
        /*
        mexPrintf("open: ");
        set<state_t*>::iterator it;
        for (it=queue.begin(); it != queue.end(); it++ )
            mexPrintf("(%d,%d) ",(*it)->x+1,(*it)->y+1);
        mexPrintf("\n");
        */
        
        state_t* s = *(queue.begin());
        queue.erase(queue.begin());
        vector<state_t*> preds;
        vector<double> costs;
        getPreds(s,preds,costs,map,mapY,mapX,mapZ);
        if(s->closed){
            mexPrintf("Error: we are expanding a state that is already closed\n");
            return;
        }
        s->closed = true;
        
        if(s->x == startX && s->y == startY && s->z == startZ){
            #if TIME_DEBUG
            t1 = clock();
            mexPrintf("plan time = %f\n", ((double)(t1-t0))/CLOCKS_PER_SEC);
            mexPrintf("path cost = %f\n", s->g);
            t0 = clock();
            #endif
            
            while(true){
                vector<state_t*> succs;
                vector<double> unused;
                getPreds(s,succs,unused,map,mapY,mapX,mapZ);
                pathX.push_back(s->x+1);
                pathY.push_back(s->y+1);
                pathZ.push_back(s->z+1);
                state_t* best_state = NULL;
                double best_cost = s->g;
                
                for(int i=0; i<succs.size(); i++){
                    if(succs[i]->x == goalX && succs[i]->y == goalY && succs[i]->z == goalZ){
                        pathX.push_back(succs[i]->x+1);
                        pathY.push_back(succs[i]->y+1);
			pathZ.push_back(succs[i]->z+1);

                        #if TIME_DEBUG
                        t1 = clock();
                        mexPrintf("path time = %f\n", ((double)(t1-t0))/CLOCKS_PER_SEC);
                        #endif

                        return;
                    }

                    if(succs[i]->g < best_cost && succs[i]->g >= 0){
                        best_state = succs[i];
                        best_cost = succs[i]->g;
                    }
                }
                if(!best_state){
                    mexPrintf("Error: path reconstruction could not find a cheaper successor\n");
                    pathX.clear();
                    pathY.clear();
	            pathZ.clear();
                    return;
                }
                s = best_state;
            }
            
        }
        
        for(int i=0; i<preds.size(); i++){
            //if(preds[i]->x==110 && preds[i]->y == 221)
                //mexPrintf("old_g=%d new_g=%d\n",preds[i]->g,s->g+1);
            if(preds[i]->g < 0 || preds[i]->g > s->g + costs[i]){
                set<state_t*>::iterator it = queue.find(preds[i]);
                if(it != queue.end()){
                    if(preds[i] != *it){
                        mexPrintf("Error: Updating the wrong element in the queue!\n");
                        return;
                    }
                    queue.erase(it);
                }
                preds[i]->g = s->g + costs[i];
                queue.insert(preds[i]);
                //mexPrintf("insert %d %d %f\n",preds[i]->x,preds[i]->y,preds[i]->g);
            }
        }
    }
    
    mexPrintf("Error: Queue is empty and we didn't find the goal\n");
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] ){ 
    double *path; 
    double *map,*robot,*target;
    mwSize mapY,mapX,mapZ;
    const int* dimension_array;
    vector<double> pathX,pathY,pathZ;
    
    //mapY = mxGetM(prhs[0]);
    //mapX = mxGetN(prhs[0]); 
    dimension_array = mxGetDimensions(prhs[0]); 
    //mexPrintf("y dimensions : %d\n",dimension_array[0]);
    //mexPrintf("y dimensions : %d\n",dimension_array[1]);
    //mexPrintf("z dimension : %d\n",dimension_array[2]);
    mapY = dimension_array[0];
    mapX = dimension_array[1];
    mapZ = dimension_array[2];
    
    map = mxGetPr(prhs[0]); 
    robot = mxGetPr(prhs[1]);
    target = mxGetPr(prhs[2]);    
    
    mexPrintf("robot(%f,%f,%f) target(%f,%f,%f)\n",robot[0],robot[1],robot[2],target[0],target[1],target[2]);
    
    runAStar(map, mapY, mapX, mapZ, robot,target,pathX,pathY,pathZ);
    
    plhs[0] = mxCreateDoubleMatrix(pathX.size(), 3, mxREAL); 
    path = mxGetPr(plhs[0]);
    
    for(int i=0; i<pathX.size(); i++){
        path[i] = pathX[i];
        path[i+pathX.size()] = pathY[i];
	path[i+2*pathX.size()] = pathZ[i];
    }
    mexPrintf("path length = %d\n",pathX.size());
    return;
}


