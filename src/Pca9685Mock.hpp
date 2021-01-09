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
#ifndef PCA9685MOCK_HPP_
#define PCA9685MOCK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "i2c_pwm/Pca9685.hpp"

namespace i2c_pwm {

class Pca9685Mock: public Pca9685
{
public:
  Pca9685Mock(const std::string &deviceFile,
              const int address,
              bool autoInitialize);
  virtual ~Pca9685Mock();

  void initialize() override;
  void allStop() override;

  void sleepMode(bool value) override;
  void setFrequencyHz(uint16_t value) override;

  uint8_t read(uint8_t reg) const override;
  void write(uint8_t reg, uint8_t data) override;
  void writeChannel(uint8_t channel, uint16_t offValue) override;
  void writeChannel(uint8_t channel, uint16_t onValue, uint16_t offValue)
    override;

private:
  rclcpp::Logger logger_;

};

}  // namespace i2c_pwm

#endif /* PCA9685MOCK_HPP_ */
