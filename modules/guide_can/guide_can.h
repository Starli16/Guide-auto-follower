#include <iostream>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/guide_can/vehicle/guide_controller.h"
#include "modules/guide_can/vehicle/guide_message_manager.h"

using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::canbus::guide::GuideMessageManager;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;
class guide_Canbus : public apollo::cyber::TimerComponent {
 public:
 private:
  int SteerEnable;
  int AccEnable;
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void PublishChassisDetail();
  void OnControl(ControlCommand& msg);
  std::unique_ptr<SocketCanClientRaw> CanClient;
  std::unique_ptr<MessageManager<ChassisDetail> > message_manager;
  CanReceiver<ChassisDetail> can_receiver;
  CanSender<ChassisDetail> can_sender;
  GuideController guide_controller;
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail> > chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<ControlCommand> >
      control_command_reader_;
};
CYBER_REGISTER_COMPONENT(guide_Canbus)
