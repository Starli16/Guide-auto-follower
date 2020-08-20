/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include "modules/common/proto/error_code.pb.h"

#include "modules/guide_can/protocol/id_0x04ef8480.h"
#include "modules/guide_can/protocol/id_0x0c040b2a.h"

#include "modules/guide_can/proto/control_command.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/guide_can/vehicle/guide_message_manager.h"
using ::apollo::canbus::ControlCommand;
using ::apollo::canbus::guide::Id0x04ef8480;
using ::apollo::canbus::guide::Id0x0c040b2a;
using ::apollo::common::ErrorCode;
using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::ProtocolData;

class GuideController {
 public:
  explicit GuideController(){};

  virtual ~GuideController();

  ::apollo::common::ErrorCode Init(
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender_,
      MessageManager<::apollo::canbus::ChassisDetail> *const message_manager);
  void ControlUpdate(ControlCommand cmd);
  void Start();
  void Stop();

 private:
  bool is_initialized_ = false;
  bool is_start = false;
  // control protocol
  CanSender<::apollo::canbus::ChassisDetail> *can_sender_;
  MessageManager<::apollo::canbus::ChassisDetail> *message_manager_;
  Id0x04ef8480 *id_0x04ef8480_ = nullptr;
  Id0x0c040b2a *id_0x0c040b2a_ = nullptr;
};
