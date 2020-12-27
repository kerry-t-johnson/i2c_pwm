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
#ifndef SERVOMANAGER_HPP_
#define SERVOMANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace i2c_pwm {

class Servo;
class Pca9685;

class ServoManager
{
public:
  ServoManager();
  virtual ~ServoManager();

  void setAll(uint16_t value);
  void setAll(float value);

  void addPca9685(uint8_t id, const std::string &deviceFile, const int address);
  void addServo(uint16_t id);
  void setServo(uint16_t id, uint16_t data);

private:
  std::vector<std::shared_ptr<Pca9685>> pcaBoards_;
  std::map<uint16_t, std::shared_ptr<Servo>> servos_;
  rclcpp::Logger logger_;
};

}  // namespace i2c_pwm

#endif  // SERVOMANAGER_HPP_
