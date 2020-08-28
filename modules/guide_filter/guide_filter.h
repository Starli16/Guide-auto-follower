#include <iostream>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/guide_can/proto/chassis_detail.pb.h"
#include "modules/guide_filter/filter.h"

using apollo::canbus::ChassisDetail;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class guide_Filter : public apollo::cyber::TimerComponent {
 public:
 private:
  ChassisDetail  chassis_detail_;
  int FilterLength;//均值滤波长度
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void PublishChassisDetail();
  void UpdateChassisDetail(const std::shared_ptr<ChassisDetail>& msg);
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail> > chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<ChassisDetail> >
      chassis_detail_origin_reader_;
};
CYBER_REGISTER_COMPONENT(guide_Filter)
