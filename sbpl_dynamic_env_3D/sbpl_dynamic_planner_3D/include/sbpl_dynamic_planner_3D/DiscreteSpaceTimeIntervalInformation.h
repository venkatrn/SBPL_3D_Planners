#ifndef DISCRETE_SPACE_TIME_INTERVAL_INFORMATION_H
#define DISCRETE_SPACE_TIME_INTERVAL_INFORMATION_H

#include <sbpl/headers.h>
#include <sbpl_dynamic_planner_3D/sbpl_dynamicObstacles.h>

class DiscreteSpaceTimeIntervalInformation : public DiscreteSpaceInformation{
public:
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV){SBPL_ERROR("ERROR: this environment requires to know if this expansion is optimal or not!\n");throw new SBPL_Exception();}
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<bool>* OptV, bool optSearch) = 0;
  virtual void Relax(int stateID){}
  virtual bool setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states=true) = 0;
  virtual bool UpdateCost(int x, int y, int z, unsigned char newcost) = 0;
  virtual int SetStart(double x, double y, double z, double theta, double startTime) = 0;
  virtual int SetGoal(double x, double y, double z, double theta) = 0;
  virtual unsigned char GetMapCost(int x, int y, int z) = 0;
  virtual void GetCoordFromState(int stateID, int& x, int& y, int& z, int& theta, int& t) const = 0;
  virtual void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath) = 0;
  virtual void getExpansions(vector<SBPL_4Dpt_t>* p){}
  virtual bool InitializeEnv(int width, int height, int depth,
                     /** if mapdata is NULL the grid is initialized to all freespace */
                     const unsigned char* mapdata,
                     double startx, double starty, double startz, double starttheta, double startTime,
                     double goalx, double goaly, double goalz, double goaltheta,
                     double goaltol_x, double goaltol_y, double goaltol_z, double goaltol_theta,
                     const vector<sbpl_3Dpt_t> & perimeterptsV,
                     double cellsize_m, double timeResolution, double temporal_padding_c,
                     double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                     unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile,
                     vector<SBPL_DynamicObstacle_t> & dynObs) = 0;
};

#endif
