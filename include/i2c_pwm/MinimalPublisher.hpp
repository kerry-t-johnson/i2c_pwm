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
#ifndef I2C_PWM__MINIMALPUBLISHER_HPP_
#define I2C_PWM__MINIMALPUBLISHER_HPP_

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher: public rclcpp::Node
{
public:
  MinimalPublisher();
  virtual ~MinimalPublisher();

private:
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::size_t count_;
};

#endif  // I2C_PWM__MINIMALPUBLISHER_HPP_
