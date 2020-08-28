#ifndef LEADERFOLLOWING_PLANNER_H
#define LEADERFOLLOWING_PLANNER_H
// PointsNumber  SampleTime
#define PointsNumber 200
#define SampleTime 0.02
#define PointDistance 0.2
#define ErrorDistance 10
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;
const float Pi = atan(1) * 4;

struct TrajSeq {
  float Seq[PointsNumber][5];  // x,y,rel_yaw,vx,yaw_rate
  vector<float> Seqv[5];
  int number;
  TrajSeq();
  void Print();
};

class Planner {
 public:
  Planner() { isInit = false; }
  bool Init(float rel_x, float rel_y, float rel_yaw);
  bool isinit();
  void traj_seq_update(float rel_x, float rel_y, float vx_1, float yaw_rate_1,
                       float last_vx_2, float last_yaw_rate_2, float delta_f_2,
                       float rel_yaw);
  // TrajSeq truth_seq_update(int timestep, TrajSeq last_truth_seq, float*
  // abs_loc_1, float* abs_loc_2, float abs_yaw_1);
  TrajSeq traj;

 private:
  bool isInit;
};

#endif  // LEADERFOLLOWING_PLANNER_H
