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
#include "Pca9685Mock.hpp"

namespace i2c_pwm {

Pca9685Mock::Pca9685Mock(const std::string& /* deviceFile */,
                         const int /* address */,
                         bool /* autoInitialize */)
  : logger_(rclcpp::get_logger("i2c_pwm.Pca9685Mock"))
{
}

Pca9685Mock::~Pca9685Mock()
{
}

void Pca9685Mock::initialize()
{
  RCLCPP_INFO(logger_, "Pca9685Mock::initialize()");
}

void Pca9685Mock::allStop()
{
  RCLCPP_INFO(logger_, "Pca9685Mock::allStop()");
}

void Pca9685Mock::sleepMode(bool value)
{
  RCLCPP_INFO(logger_,
              "Pca9685Mock::sleepMode(value=%s)",
              value ? "true" : "false");
}

void Pca9685Mock::setFrequencyHz(uint16_t value)
{
  RCLCPP_INFO(logger_, "Pca9685Mock::setFrequencyHz(value=%d)", value);
}

uint8_t Pca9685Mock::read(uint8_t reg) const
{
  RCLCPP_INFO(logger_, "Pca9685Mock::read(reg=%d)", reg);
  return reg;
}

void Pca9685Mock::write(uint8_t reg, uint8_t data)
{
  RCLCPP_INFO(logger_, "Pca9685Mock::write(reg=%d, data=%d)", reg, data);
}

void Pca9685Mock::writeChannel(uint8_t channel, uint16_t offValue)
{
  RCLCPP_INFO(logger_,
              "Pca9685Mock::writeChannel(channel=%d, offValue=%d)",
              channel,
              offValue);
}

void Pca9685Mock::writeChannel(uint8_t channel,
                               uint16_t onValue,
                               uint16_t offValue)
{
  RCLCPP_INFO(logger_,
              "Pca9685Mock::writeChannel(channel=%d, onValue=%d, offValue=%d)",
              channel,
              onValue,
              offValue);
}

}  // namespace i2c_pwm
