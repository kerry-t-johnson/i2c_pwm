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
#include <Pca9685.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Servo.hpp>
#include <algorithm>
#include <memory>

namespace {
const uint16_t INVALID_VALUE = i2c_pwm::Pca9685::MAX_VALUE + 1;

}  // namespace

namespace i2c_pwm {

Servo::Servo(std::shared_ptr<Pca9685> pcaBoard, uint16_t id)
  : pcaBoard_(pcaBoard),
    id_(id),
    center_(Pca9685::MAX_VALUE / 2),
    range_(Pca9685::MAX_VALUE),
    invertDirection_(false),
    value_(0),
    logger_(rclcpp::get_logger("i2c_pwm.Servo"))
{
}

Servo::~Servo()
{
}

uint16_t Servo::id() const
{
  return id_;
}

uint16_t Servo::center() const
{
  return center_;
}

uint16_t Servo::range() const
{
  return range_;
}

bool Servo::invertDirection() const
{
  return invertDirection_;
}

uint16_t Servo::value() const
{
  return value_;
}

void Servo::configure(uint16_t center, uint16_t range, bool invertDirection)
{
  uint16_t newCenter = std::min(center, Pca9685::MAX_VALUE);
  if (newCenter != center) {
    RCLCPP_WARN(logger_,
                "[Servo %d] Invalid center value: %d (must be in the interval [0, %d])",
                id_,
                center,
                Pca9685::MAX_VALUE);
  }

  const uint16_t newRange = std::min(range, Pca9685::MAX_VALUE);
  if (newRange != range) {
    RCLCPP_WARN(logger_,
                "[Servo %d] Invalid range value: %d (must be in the interval [0, %d])",
                id_,
                range,
                Pca9685::MAX_VALUE);
  }

  const float halfRange = newRange / 2.0;
  if (newCenter - halfRange < Pca9685::MIN_VALUE
      || halfRange + newCenter > Pca9685::MAX_VALUE) {
    RCLCPP_WARN(logger_,
                "[Servo %d] Invalid range/center combination %d/%d (must be in the interval [0, %d])",
                id_,
                newCenter,
                newRange,
                Pca9685::MAX_VALUE);

    newCenter = newRange / 2;
  }

  center_ = newCenter;
  range_ = newRange;
  invertDirection_ = invertDirection;

  RCLCPP_INFO(logger_,
              "[Servo %d] Configured: center=%d, range=%d, direction=%d",
              id_,
              center_,
              range_,
              invertDirection_ ? -1 : 1);
}

void Servo::set(uint16_t value)
{
  const uint16_t clampedValue = std::min(value, Pca9685::MAX_VALUE);
  if (value > Pca9685::MAX_VALUE) {
    RCLCPP_WARN(logger_,
                "[Servo %d] Clamped value %d to %d instead",
                id_,
                value,
                clampedValue);
  }

  value_ = clampedValue;

  pcaBoard_->writeChannel(id_ % Pca9685::NUM_CHANNELS, value_);
}

void Servo::set(float value)
{
  const float clampedValue = std::max(std::min(value, 1.0f), -1.0f);

  if (clampedValue != value) {
    RCLCPP_WARN(logger_,
                "[Servo %d] Clamped value %f to %f instead",
                id_,
                value,
                clampedValue);
  }

  const float halfRange = range_ / 2.0;
  const uint16_t pos = ((invertDirection_ ? -1 : 1) * (halfRange * clampedValue))
                       + center_;

  RCLCPP_INFO(logger_,
              "[Servo %d] Converted proportional value to absolute: %f --> %d",
              id_,
              value,
              pos);

  set(pos);
}

}  // namespace i2c_pwm
