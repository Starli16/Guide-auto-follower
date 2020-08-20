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

#include "modules/guide_can/protocol/id_0x18f02501.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

Id0x18f02501::Id0x18f02501() {}
const int32_t Id0x18f02501::ID = 0x18F02501;

void Id0x18f02501::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_x_speed(x_speed(bytes, length));
  chassis->set_x_acc(x_acc(bytes, length));
}

// config detail: {'name': 'x_speed', 'offset': 0.0, 'precision': 0.1, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|80]', 'bit': 24, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'm/s'}
double Id0x18f02501::x_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'x_acc', 'offset': -15.0, 'precision': 0.1, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[-15|15]', 'bit': 8, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'm/s^2'}
double Id0x18f02501::x_acc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -15.000000;
  return ret;
}
}  // namespace guide
}  // namespace canbus
}  // namespace apollo
