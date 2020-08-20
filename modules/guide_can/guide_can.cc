#include "guide_can.h"

#include <iostream>
using std::cout;
using std::endl;


bool guide_Canbus::Init() {
  CanClient = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CANCardParameter CanPara = CANCardParameter();
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient->Init(CanPara);
  CanClient->Start();  // open CAN0

  message_manager =
      std::unique_ptr<MessageManager<ChassisDetail> >(new GuideMessageManager);
  message_manager->ClearSensorData();
  // Init message_manager

  can_receiver.Init(CanClient.get(), message_manager.get(), 1);
  can_receiver.Start();
  can_sender.Init(CanClient.get(), 1);
  can_sender.Start();
  // Init receiver sender
  guide_controller.Init(&can_sender, message_manager.get());
  guide_controller.Start();
  // Init Controller
  AINFO << "Canbus Init";

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetail");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "guide/ControlCommand",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });

  return true;
}

bool guide_Canbus::Proc() {  // Timer callback
  PublishChassisDetail();
  return true;
}
void guide_Canbus::Clear() {  // shutdown
  guide_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  CanClient->Stop();
  // std::cout<<"stopping and clearing"<<std::endl;
}
void guide_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata;
  message_manager->GetSensorData(&sensordata);

  AINFO << "uwb distance is :" << sensordata.uwb_distance() << endl;
  AINFO << "uwb azimuth is :" << sensordata.uwb_azimuth() << endl;
  ADEBUG << sensordata.DebugString();
  chassis_detail_writer_->Write(sensordata);
  return;
}
void guide_Canbus::OnControl(
    ControlCommand&
        msg) {  // control callback function  will move to reader callback
  static ControlCommand cmd;
  cmd.set_control_steer(msg.control_steer());
  cmd.set_control_acc(msg.control_acc());

  guide_controller.ControlUpdate(cmd);
  can_sender.Update();
  return;
}
