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

#include "modules/guide_can/vehicle/guide_controller.h"

ErrorCode GuideController::Init(
    CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "GuideController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  id_0x04ef8480_ = dynamic_cast<Id0x04ef8480 *>(
      message_manager_->GetMutableProtocolDataById(Id0x04ef8480::ID));
  if (id_0x04ef8480_ == nullptr) {
    AERROR << "Id0x04ef8480 does not exist in the GuideMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  id_0x0c040b2a_ = dynamic_cast<Id0x0c040b2a *>(
      message_manager_->GetMutableProtocolDataById(Id0x0c040b2a::ID));
  if (id_0x0c040b2a_ == nullptr) {
    AERROR << "Id0x0c040b2a does not exist in the GuideMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Id0x04ef8480::ID, id_0x04ef8480_, false);
  can_sender_->AddMessage(Id0x0c040b2a::ID, id_0x0c040b2a_, false);

  // need sleep to ensure all messages received
  AINFO << "GuideController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

GuideController::~GuideController() = default;

void GuideController::Start() {
  if (!is_initialized_) {
    AERROR << "Controller didn't Init";
    return;
  }
  if (!is_start)
    is_start = true;
  else
    AERROR << "Duplicate Start ";
}
void GuideController::Stop() {
  if (!is_initialized_) {
    AERROR << "Controller didn't Init";
    return;
  }
  if (is_start) is_start = false;
}

void GuideController::ControlUpdate(ControlCommand cmd) {
  if (!is_start) {
    AERROR << "Controller didn't start";
    return;
  }

  id_0x04ef8480_->set_control_steer(cmd.control_steer());
  id_0x0c040b2a_->set_control_acc(cmd.control_acc());
}
