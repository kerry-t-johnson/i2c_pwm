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
#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <memory>

namespace i2c_pwm {

class Pca9685;

class Servo
{
public:
  Servo(std::shared_ptr<Pca9685> pcaBoard, uint16_t id);
  virtual ~Servo();

  uint16_t id() const;

  uint16_t center() const;

  uint16_t range() const;

  bool invertDirection() const;

  uint16_t value() const;

  void configure(uint16_t center, uint16_t range, bool invertDirection);

  /**
   * @brief Sets the absolute value of the servo in the interval [0, 4096]
   *
   * When working with a continuous rotation servo, sets the speed of the servo.
   *
   * When working with a fixed 180 degree rotation servo, sets the angle of
   * the servo.
   *
   * Hint: setting the servo pulse value to zero (0) causes the servo to power
   * off. This is referred to as 'coast'. setting a servo to its center value
   * leaves the servo powered and is referred to as 'brake'.
   *
   * @param value the absolute value of the servo, in the interval [0, 4096]
   */
  void set(uint16_t value);

  /**
   * @brief Sets the proportional value of the servo in the interval [-1.0, 1.0]
   *
   * When working with a continuous rotation servo, sets the speed of the servo.
   *
   * When working with a fixed 180 degree rotation servo, sets the angle of
   * the servo.
   *
   * @param value the proportional value of the servo, in the interval [-1.0, 1.0]
   */
  void set(float value);

private:
  std::shared_ptr<Pca9685> pcaBoard_;
  const uint16_t id_;
  uint16_t center_;
  uint16_t range_;
  bool invertDirection_;
  uint16_t value_;
  rclcpp::Logger logger_;
};

}  // namespace i2c_pwm

#endif  // SERVO_HPP_
