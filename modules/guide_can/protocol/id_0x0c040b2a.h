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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/guide_can/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace guide {

class Id0x0c040b2a : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c040b2a();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Control_acc', 'offset': -15.0, 'precision': 0.1,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[-15|15]', 'bit': 0,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
  Id0x0c040b2a* set_control_acc(double control_acc);

 private:
  // config detail: {'name': 'Control_acc', 'offset': -15.0, 'precision': 0.1,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[-15|15]', 'bit': 0,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
  void set_p_control_acc(uint8_t* data, double control_acc);

 private:
  double control_acc_;
};

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
