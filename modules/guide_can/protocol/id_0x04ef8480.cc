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

#include "modules/guide_can/protocol/id_0x04ef8480.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x04ef8480::ID = 0x04EF8480;

// public
Id0x04ef8480::Id0x04ef8480() { Reset(); }

uint32_t Id0x04ef8480::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x04ef8480::UpdateData(uint8_t* data) {
  set_p_control_steer(data, control_steer_);
}

void Id0x04ef8480::Reset() {
  // TODO(All) :  you should check this manually
  control_steer_ = 0.0;
}

Id0x04ef8480* Id0x04ef8480::set_control_steer(double control_steer) {
  control_steer_ = control_steer;
  return this;
}

// config detail: {'name': 'Control_steer', 'offset': -3276.7, 'precision': 0.1,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[-880|880]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
void Id0x04ef8480::set_p_control_steer(uint8_t* data, double control_steer) {
  control_steer = ProtocolData::BoundedValue(-880.0, 880.0, control_steer);
  int x = (control_steer - -3276.700000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
