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

#include "modules/guide_can/protocol/id_0x0c040b2a.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c040b2a::ID = 0x0C040B2A;

// public
Id0x0c040b2a::Id0x0c040b2a() { Reset(); }

uint32_t Id0x0c040b2a::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c040b2a::UpdateData(uint8_t* data) {
  set_p_control_acc(data, control_acc_);
}

void Id0x0c040b2a::Reset() {
  // TODO(All) :  you should check this manually
  control_acc_ = 0.0;
}

Id0x0c040b2a* Id0x0c040b2a::set_control_acc(double control_acc) {
  control_acc_ = control_acc;
  return this;
}

// config detail: {'name': 'Control_acc', 'offset': -15.0, 'precision': 0.1,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[-15|15]', 'bit': 0,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
void Id0x0c040b2a::set_p_control_acc(uint8_t* data, double control_acc) {
  control_acc = ProtocolData::BoundedValue(-15.0, 15.0, control_acc);
  int x = (control_acc - -15.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);

  // set mode
  int mode;
  if(control_acc == 0) mode=0;
  else if(control_acc < 0) mode = 1;
  else if(control_acc > 0) mode = 2;
  t = (mode <<4) & 0xFF;
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 8);


  // set cycle 
  static int p=0;
  p=p+1;
  if(p>15) p=0;
  t = p & 0xFF;
  Byte to_set3(data + 7);
  to_set3.set_value(t, 0, 8);

}

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
