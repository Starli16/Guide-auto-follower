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

#include "modules/guide_can/protocol/id_0x18ff4bd1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

Id0x18ff4bd1::Id0x18ff4bd1() {}
const int32_t Id0x18ff4bd1::ID = 0x18FF4BD1;

void Id0x18ff4bd1::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_steer_angle(steer_angle(bytes, length));
}

// config detail: {'name': 'steer_angle', 'offset': -3276.7, 'precision': 0.1,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[-1260|1260]', 'bit':
// 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
double Id0x18ff4bd1::steer_angle(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -3276.700000;
  return ret;
}
}  // namespace guide
}  // namespace canbus
}  // namespace apollo
