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
#ifndef I2C_PWM__MINIMALSUBSCRIBER_HPP_
#define I2C_PWM__MINIMALSUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber: public rclcpp::Node
{
public:
  MinimalSubscriber();
  virtual ~MinimalSubscriber();

private:
  void topicCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

#endif  // I2C_PWM__MINIMALSUBSCRIBER_HPP_
