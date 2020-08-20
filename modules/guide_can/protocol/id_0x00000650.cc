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

#include "modules/guide_can/protocol/id_0x00000650.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace guide {

using ::apollo::drivers::canbus::Byte;

Id0x00000650::Id0x00000650() {}
const int32_t Id0x00000650::ID = 0x00000650;

void Id0x00000650::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_uwb_attitude(uwb_attitude(bytes, length));
  chassis->set_uwb_azimuth(uwb_azimuth(bytes, length));
  chassis->set_uwb_distance(uwb_distance(bytes, length));
  chassis->set_uwb_status(uwb_status(bytes, length));
}

// config detail: {'name': 'uwb_zitai', 'offset': 0.0, 'precision': 1.0, 'len':
// 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 48, 'type':
// 'int', 'order': 'intel', 'physical_unit': 'deg'}
int Id0x00000650::uwb_attitude(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'uwb_fangwei', 'offset': 0.0, 'precision': 1.0,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 32,
// 'type': 'int', 'order': 'intel', 'physical_unit': 'deg'}
int Id0x00000650::uwb_azimuth(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'uwb_distance', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'm'}
double Id0x00000650::uwb_distance(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'uwb_status', 'offset': 0.0, 'precision': 1.0, 'len':
// 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x00000650::uwb_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace guide
}  // namespace canbus
}  // namespace apollo
