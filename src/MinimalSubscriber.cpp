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
#include <i2c_pwm/MinimalSubscriber.hpp>
#include <functional>

MinimalSubscriber::MinimalSubscriber()
  : rclcpp::Node("MinimalSubscriber")
{
  subscriber_ = this->create_subscription<std_msgs::msg::String>("topic",
                                                                 10,
                                                                 std::bind(&MinimalSubscriber::topicCallback,
                                                                           this,
                                                                           std::placeholders::_1));
}

MinimalSubscriber::~MinimalSubscriber()
{
}

void MinimalSubscriber::topicCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}
