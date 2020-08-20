#include <iostream>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "Planner.h"
#include "modules/guide_can/proto/chassis_detail.pb.h"
#include "modules/guide_planner/proto/Trajectory.pb.h"

using apollo::canbus::ChassisDetail;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::Writer;
using apollo::planner::TrajInfo;

class guide_Planner : public apollo::cyber::Component<ChassisDetail> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ChassisDetail>& msg0) override;

 private:
  //  std::shared_ptr<Writer<ControlCommand> > writer;
  Planner planner;
  void TrajUpdate(const std::shared_ptr<ChassisDetail>& msg0);
  void WriteTraj();
  std::shared_ptr<Writer<TrajInfo> > writer;
};
CYBER_REGISTER_COMPONENT(guide_Planner)
