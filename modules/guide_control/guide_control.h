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
  float steer_PID_kp,steer_PID_ki,steer_PID_kd;
  float BezierX[251], BezierY[251];
  ControlCommand controlcmd;
  float Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0,const std::shared_ptr<TrajInfo>& msg1);
  float Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0);
  void DealTraj(const std::shared_ptr<TrajInfo>& msg1);
  void BezierFitting(const std::shared_ptr<TrajInfo>& msg1);
  float CalBezierLoc(int n, float t, float p[]);
  int FindLookAheadPoint(float LookAheadDis, const std::shared_ptr<TrajInfo>& msg1);
  int FindLookAheadPointBezier(float LookAheadDis);
};
CYBER_REGISTER_COMPONENT(guide_Control)
