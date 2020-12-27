// Copyright 2020 Kerry Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef PCA9685_HPP_
#define PCA9685_HPP_

#include <stdint.h>
#include <string>

namespace i2c_pwm {

class Pca9685
{
public:
  static const uint8_t NUM_CHANNELS = 16;
  static const uint16_t MIN_VALUE = 0;
  static const uint16_t MAX_VALUE = 4096;  // 2**12

  virtual ~Pca9685()
  {
  }

  virtual void sleepMode(bool value) = 0;
  virtual void setFrequencyHz(uint16_t value) = 0;

  virtual uint8_t read(uint8_t reg) const = 0;
  virtual void write(uint8_t reg, uint8_t data) = 0;
  virtual void writeChannel(uint8_t channel, uint16_t data) = 0;
};

}  // namespace i2c_pwm

#endif  // PCA9685_HPP_
