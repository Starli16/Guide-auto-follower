#include <iostream>
#include <vector>

#include "controller/PIDcontroller.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/guide_can/proto/chassis_detail.pb.h"
#include "modules/guide_can/proto/control_command.pb.h"
#include "modules/guide_planner/proto/Trajectory.pb.h"

using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::Writer;
using apollo::planner::TrajInfo;
class guide_Control : public apollo::cyber::Component<ChassisDetail, TrajInfo> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ChassisDetail>& msg0,
            const std::shared_ptr<TrajInfo>& msg1) override;

 private:
  std::shared_ptr<Writer<ControlCommand> > writer;
  float control_acc = 0;
  float control_steer = 0;
  float lookahead_x = 0;
  float lookahead_y = 0;
  float err_lat = 0;
  float BezierX[251], BezierY[251];
  ControlCommand controlcmd;
  float Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0,const std::shared_ptr<TrajInfo>& msg1);
  float Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0);
  void DealTraj(const std::shared_ptr<TrajInfo>& msg1);
  void BezierFitting(const std::shared_ptr<TrajInfo>& msg1);
  float CalBezierLoc(int n, float t, float p[]);
  int FindLookAheadPoint(float LookAheadDis, const std::shared_ptr<TrajInfo>& msg1);
  int FindLookAheadPointBezier(float LookAheadDis);
  void ReadConfig();
  struct ConfigInfo{
    float Speed[4]; // 文件中输入km/h 程序转换成m/s 
    float DesiredDistance; //m
    float SteerKp[3],SteerKi[3],SteerKd[3];
    float LookAheadDistance[3]; // m
    float SteerPIDProportion; 
    float AccLimit[2]; // m/s^2
    float AccKd[3],AccKv[3],AccKa[3];
    float DeltaS[2]; // m
    float PedalEffect; 
    float kAcc,kBrake,bAcc,bBrake;
    float AccCorrectMax,BrakeCorrectMax; // m/s^2
  }configinfo;
};
CYBER_REGISTER_COMPONENT(guide_Control)
