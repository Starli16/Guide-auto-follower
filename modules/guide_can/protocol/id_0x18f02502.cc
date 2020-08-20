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

#include "modules/guide_can/protocol/id_0x18f02502.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

Id0x18f02502::Id0x18f02502() {}
const int32_t Id0x18f02502::ID = 0x18F02502;

void Id0x18f02502::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_pedal_acc(pedal_acc(bytes, length));
  chassis->set_pedal_brake(pedal_brake(bytes, length));
}

// config detail: {'name': 'pedal_acc', 'offset': 0.0, 'precision': 0.1, 'len':
// 32, 'is_signed_var': True, 'physical_range': '[0|200]', 'bit': 16, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'deg'}
double Id0x18f02502::pedal_acc(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 3);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 2);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'pedal_brake', 'offset': 0.0, 'precision': 0.1,
// 'len': 32, 'is_signed_var': True, 'physical_range': '[0|200]', 'bit': 0,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
double Id0x18f02502::pedal_brake(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 1);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 0);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 0;
  x >>= 0;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace guide
}  // namespace canbus
}  // namespace apollo
