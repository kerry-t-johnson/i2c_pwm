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
#include <Pca9685Impl.hpp>
#include <Servo.hpp>
#include <ServoManager.hpp>
#include <memory>
#include <string>

namespace {
const uint8_t MAX_BOARDS = 62;
}  // namespace

namespace i2c_pwm {

ServoManager::ServoManager()
  : pcaBoards_(MAX_BOARDS), logger_(rclcpp::get_logger("i2c_pwm.ServoManager"))
{
}

ServoManager::~ServoManager()
{
}

void ServoManager::addPca9685(uint8_t id,
                              const std::string &deviceFile,
                              const int address)
{
  if (pcaBoards_[id]) {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << id << " already exists";
    throw std::runtime_error(ostr.str());
  }

  std::shared_ptr<Pca9685> pcaBoard = std::shared_ptr<Pca9685>(new Pca9685Impl(deviceFile,
                                                                               address));
  pcaBoards_[id] = pcaBoard;

  RCLCPP_INFO(logger_, "Created PCA9685 board at slot %d", id);
}

void ServoManager::addServo(uint16_t id)
{
  if (servos_.count(id)) {
    std::ostringstream ostr;
    ostr << "Servo " << id << " already exists";
    throw std::runtime_error(ostr.str());
  }

  const uint8_t boardId = id / Pca9685::NUM_CHANNELS;

  std::shared_ptr<Pca9685> pcaBoard = pcaBoards_[boardId];
  if (!pcaBoard) {
    std::ostringstream ostr;
    ostr << "PCA9685 board " << boardId << " does not exist";
    throw std::runtime_error(ostr.str());
  }

  std::shared_ptr<Servo> servo(new Servo(pcaBoard, id));
  servos_[id] = servo;

  RCLCPP_INFO(logger_,
              "Created Servo %d (Board: %d, Channel: %d",
              id,
              boardId,
              id % Pca9685::NUM_CHANNELS);
}

void ServoManager::setAll(uint16_t value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter) {
    iter->second->set(value);
  }
}

void ServoManager::setAll(float value)
{
  for (auto iter = servos_.begin(); iter != servos_.end(); ++iter) {
    iter->second->set(value);
  }
}

}  // namespace i2c_pwm
