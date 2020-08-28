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

#include "modules/guide_can/vehicle/guide_message_manager.h"

#include "modules/guide_can/protocol/id_0x04ef8480.h"
#include "modules/guide_can/protocol/id_0x0c040b2a.h"

#include "modules/guide_can/protocol/id_0x0000005A.h"
#include "modules/guide_can/protocol/id_0x00000650.h"
#include "modules/guide_can/protocol/id_0x03100000.h"
#include "modules/guide_can/protocol/id_0x18f02501.h"
#include "modules/guide_can/protocol/id_0x18f02502.h"
#include "modules/guide_can/protocol/id_0x18ff4bd1.h"

namespace apollo {
namespace canbus {
namespace guide {

GuideMessageManager::GuideMessageManager() {
  // Control Messages
  AddSendProtocolData<Id0x04ef8480, true>();
  AddSendProtocolData<Id0x0c040b2a, true>();
  ::apollo::drivers::canbus::ProtocolData<apollo::canbus::ChassisDetail>
      *id8480 = GetMutableProtocolDataById(0x04ef8480);
  id8480->SetDataLength(4);
  // set steer data_length

  // Report Messages
  AddRecvProtocolData<Id0x00000650, true>();
  AddRecvProtocolData<Id0x0000005A, true>();
  AddRecvProtocolData<Id0x03100000, true>();
  AddRecvProtocolData<Id0x18f02501, true>();
  AddRecvProtocolData<Id0x18f02502, true>();
  AddRecvProtocolData<Id0x18ff4bd1, true>();
}

GuideMessageManager::~GuideMessageManager() {}

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
