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
#include "i2c_pwm/Pca9685.hpp"
#include "Pca9685Impl.hpp"
#include "Pca9685Mock.hpp"

namespace {
bool mockMode = false;
} // namespace

namespace i2c_pwm {

const uint8_t Pca9685::NUM_CHANNELS;
const uint16_t Pca9685::MIN_VALUE;
const uint16_t Pca9685::MAX_VALUE;

std::shared_ptr<Pca9685> Pca9685::create(const std::string &deviceFile,
                                         const int address,
                                         bool autoInitialize)
{
  if (mockMode) {
    return std::shared_ptr<Pca9685>(new Pca9685Mock(deviceFile,
                                                    address,
                                                    autoInitialize));
  } else {
    return std::shared_ptr<Pca9685>(new Pca9685Impl(deviceFile,
                                                    address,
                                                    autoInitialize));
  }
}

void Pca9685::setMockMode(bool value)
{
  mockMode = value;
}

}
