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
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <utility>
#include "Pca9685.hpp"
#include "Servo.hpp"

namespace {
class MockPca9685: public i2c_pwm::Pca9685
{
public:
  MockPca9685()
    : channel(0), value(0)
  {
  }

  virtual ~MockPca9685()
  {
  }

  void sleepMode(__attribute__((unused)) bool _)
  {
  }

  void setFrequencyHz(uint16_t)
  {
  }

  uint8_t read(uint8_t reg) const
  {
    return reg;
  }

  void write(uint8_t, uint8_t)
  {
  }

  void writeChannel(uint8_t inChannel, uint16_t inValue)
  {
    this->channel = inChannel;
    this->value = inValue;
  }

  uint8_t channel;
  uint16_t value;
};
}  // namespace

namespace i2c_pwm {

TEST(ServoTests, testConstructor)
{
  const uint16_t EXPECTED_ID = 1;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  EXPECT_EQ(uut.id(), EXPECTED_ID);
  EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
  EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
  EXPECT_EQ(uut.invertDirection(), false);
  EXPECT_EQ(uut.value(), 0);
}

TEST(ServoTests, testConfigure)
{
  const uint16_t EXPECTED_ID = 1;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  uut.configure(Pca9685::MAX_VALUE / 2, Pca9685::MAX_VALUE, false);
  EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
  EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
  EXPECT_EQ(uut.invertDirection(), false);
}

TEST(ServoTests, testConfigureOutOfRange)
{
  const uint16_t EXPECTED_ID = 1;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  uut.configure(Pca9685::MAX_VALUE + 1, Pca9685::MAX_VALUE + 1, false);
  EXPECT_EQ(uut.center(), Pca9685::MAX_VALUE / 2);
  EXPECT_EQ(uut.range(), Pca9685::MAX_VALUE);
  EXPECT_EQ(uut.invertDirection(), false);
}

TEST(ServoTests, testSetAbsolute)
{
  const uint16_t EXPECTED_ID = 17;
  const uint16_t EXPECTED_CHANNEL = 1;  // 17 % 16
  const uint16_t VALUE = 4;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  uut.set(VALUE);

  EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
  EXPECT_EQ(mockPca9685->value, VALUE);
}

TEST(ServoTests, testSetAbsoluteInvalid)
{
  const uint16_t EXPECTED_ID = 17;
  const uint16_t EXPECTED_CHANNEL = 1;  // 17 % 16
  const uint16_t VALUE = Pca9685::MAX_VALUE + 42;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  uut.set(VALUE);

  EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
  EXPECT_EQ(mockPca9685->value, Pca9685::MAX_VALUE);
}

TEST(ServoTests, testSetProportional)
{
  //  |-----------|-----------|
  // Min        Center       Max
  //               <---------->
  //                Half-range
  const uint16_t HALF_RANGE = Pca9685::MAX_VALUE / 2;
  const float STEP_SIZE_PROPORTIONAL = 0.25f;
  const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

  const uint16_t EXPECTED_ID = 35;
  const uint16_t EXPECTED_CHANNEL = 3;  // 35 % 16
  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  for (int i = 0; i <= Pca9685::MAX_VALUE / STEP_SIZE_ABSOLUTE; ++i) {
    uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->value,
      Pca9685::MIN_VALUE + (i * STEP_SIZE_ABSOLUTE)) << "Step: " << i;
  }
}

TEST(ServoTests, testSetProportionalCustomRange)
{
  //  |-----------|-----------|
  // Min        Center       Max
  //               <---------->
  //                Half-range
  const uint16_t CUSTOM_RANGE = 1000;
  const uint16_t HALF_RANGE = CUSTOM_RANGE / 2;
  const float STEP_SIZE_PROPORTIONAL = 0.25f;
  const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

  const uint16_t EXPECTED_ID = 35;
  const uint16_t EXPECTED_CHANNEL = 3;  // 35 % 16
  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);
  uut.configure(CUSTOM_RANGE / 2, CUSTOM_RANGE, false);

  for (int i = 0; i <= CUSTOM_RANGE / STEP_SIZE_ABSOLUTE; ++i) {
    uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->value,
      Pca9685::MIN_VALUE + (i * STEP_SIZE_ABSOLUTE)) << "Step: " << i;
  }
}

TEST(ServoTests, testSetProportionalInverted)
{
  //  |-----------|-----------|
  // Min        Center       Max
  //               <---------->
  //                Half-range
  const uint16_t HALF_RANGE = Pca9685::MAX_VALUE / 2;
  const float STEP_SIZE_PROPORTIONAL = 0.25f;
  const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

  const uint16_t EXPECTED_ID = 35;
  const uint16_t EXPECTED_CHANNEL = 3;  // 35 % 16

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);
  uut.configure(Pca9685::MAX_VALUE / 2, Pca9685::MAX_VALUE, true);

  for (int i = 0; i <= Pca9685::MAX_VALUE / STEP_SIZE_ABSOLUTE; ++i) {
    uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->value,
      Pca9685::MAX_VALUE - (i * STEP_SIZE_ABSOLUTE)) << "Step: " << i;
  }
}

TEST(ServoTests, testSetProportionalInvertedCustomRange)
{
  //  |-----------|-----------|
  // Min        Center       Max
  //               <---------->
  //                Half-range
  const uint16_t CUSTOM_RANGE = 1000;
  const uint16_t HALF_RANGE = CUSTOM_RANGE / 2;
  const float STEP_SIZE_PROPORTIONAL = 0.25f;
  const uint16_t STEP_SIZE_ABSOLUTE = HALF_RANGE * STEP_SIZE_PROPORTIONAL;

  const uint16_t EXPECTED_ID = 35;
  const uint16_t EXPECTED_CHANNEL = 3;  // 35 % 16
  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);
  uut.configure(CUSTOM_RANGE / 2, CUSTOM_RANGE, true);

  for (int i = 0; i <= CUSTOM_RANGE / STEP_SIZE_ABSOLUTE; ++i) {
    uut.set(-1.0f + (i * STEP_SIZE_PROPORTIONAL));
    EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
    EXPECT_EQ(mockPca9685->value,
      CUSTOM_RANGE - (i * STEP_SIZE_ABSOLUTE)) << "Step: " << i;
  }
}

TEST(ServoTests, testSetProportionalInvalid)
{
  const uint16_t EXPECTED_ID = 17;
  const uint16_t EXPECTED_CHANNEL = 1;  // 17 % 16
  const uint16_t VALUE = Pca9685::MAX_VALUE + 42;

  std::shared_ptr<MockPca9685> mockPca9685(new MockPca9685());

  Servo uut(mockPca9685, EXPECTED_ID);

  uut.set(VALUE);

  EXPECT_EQ(mockPca9685->channel, EXPECTED_CHANNEL);
  EXPECT_EQ(mockPca9685->value, Pca9685::MAX_VALUE);
}

}  // namespace i2c_pwm

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

// initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

// shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
