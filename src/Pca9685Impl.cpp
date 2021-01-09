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
#include "Pca9685Impl.hpp"
#include <fcntl.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
}
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <bitset>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

// NOTES:
//
// Right now, this implementation assumes a single I2C board on the
// system (or, more accurately, a single user of the I2C device file).
// Changes would need to be made in order to account for multiple I2C
// users (e.g. convert each Pca9685 to a *Node* or convert this file
// to behave properly in the presence of multiple users... non-blocking
// calls, etc).
//
//    See: https://www.spinics.net/lists/linux-i2c/msg17264.html
//    And: https://stackoverflow.com/a/41187358
//
// For hardware specs, register values, and general board behavior:
//    Ref: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf

namespace {

const float OSC_CLOCK_HZ = 25000000.0;  // 25MHz
const uint16_t MAX_FREQ_MOD_HZ = 1526;
const uint16_t MIN_FREQ_MOD_HZ = 24;
const uint16_t DEFAULT_FREQ_HZ = 50;

const uint8_t REGISTER_MODE1 = 0x00;
const uint8_t REGISTER_MODE2 = 0x01;
const uint8_t REGISTER_CHANNEL0_ON_LOW = 0x06;
const uint8_t REGISTER_CHANNEL0_ON_HIGH = 0x07;
const uint8_t REGISTER_CHANNEL0_OFF_LOW = 0x08;
const uint8_t REGISTER_CHANNEL0_OFF_HIGH = 0x09;
const uint8_t REGISTER_ALL_OFF_HIGH = 0xFD;
const uint8_t REGISTER_PRE_SCALE = 0xFE;

const uint8_t RESTART = 0x80;
const uint8_t CLEAR_RESTART = 0x7F;
const uint8_t SLEEP = 0x10;
const uint8_t ALL_FULL_OFF = 0x10;

// By default, we only want ALLCALL to be enabled (bit 0):
const uint8_t DEFAULT_MODE1_MASK = 0x01;

// By default, we only want OUTDRV (totem) to be enabled (bit 2);
const uint8_t DEFAULT_MODE2_MASK = 0x04;

// Documentation states "at least" 500us
const struct timespec OSC_STABILIZATION_DELAY = { 0, 600000L };

const uint8_t LOWER_BYTE_MASK = 0xFF;
const uint8_t UPPER_BYTE_SHIFT = 8;

}  // namespace

namespace i2c_pwm {

Pca9685Impl::Pca9685Impl(const std::string &deviceFile,
                         const int address,
                         bool autoInitialize)
  : deviceFilePath_(deviceFile),
    address_(address),
    fd_(0),
    frequencyHz_(0),
    frequencyScale_(1.0f),
    logger_(rclcpp::get_logger("i2c_pwm.Pca9685"))
{
  if (autoInitialize) {
    Pca9685Impl::initialize();
  }
}

Pca9685Impl::~Pca9685Impl()
{
  if (fd_ != 0) {
    ::close(fd_);
  }
}

void Pca9685Impl::initialize()
{
  if (fd_ == 0) {
    fd_ = ::open(deviceFilePath_.c_str(), O_RDWR);  // | O_NONBLOCK);

    if (fd_ < 0) {
      std::ostringstream ostr;
      ostr << "Unable to open I2C device file: " << deviceFilePath_;
      throw std::runtime_error(ostr.str());
    }

    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
      std::ostringstream ostr;
      ostr << "Unable to set I2C address: 0x" << std::hex << address_;
      throw std::runtime_error(ostr.str());
    }

    write(REGISTER_MODE1, DEFAULT_MODE1_MASK);
    write(REGISTER_MODE2, DEFAULT_MODE2_MASK);

    setFrequencyHz(DEFAULT_FREQ_HZ * 2);

    allStop();
  }
}

void Pca9685Impl::allStop()
{
  write(REGISTER_ALL_OFF_HIGH, ALL_FULL_OFF);
}

void Pca9685Impl::sleepMode(bool value)
{
  const uint8_t original_mode = read(REGISTER_MODE1);
  const uint8_t new_mode = original_mode & (value ? SLEEP : ~SLEEP);

  write(REGISTER_MODE1, new_mode);

  nanosleep(&OSC_STABILIZATION_DELAY, NULL);
}

void Pca9685Impl::setFrequencyHz(uint16_t value)
{
  const uint16_t clampedFreqModHz = std::max(MIN_FREQ_MOD_HZ,
                                             std::min(MAX_FREQ_MOD_HZ, value));

  if (clampedFreqModHz != value) {
    RCLCPP_ERROR(logger_,
                 "Clamped frequency modulation value %dHz to %dHz instead",
                 value,
                 clampedFreqModHz);
  }

  frequencyHz_ = clampedFreqModHz;

  frequencyScale_ = static_cast<float>(frequencyHz_)
                    / static_cast<float>(DEFAULT_FREQ_HZ);

  //                  ( OSC_CLOCK_HZ )
  // prescale = round( -------------- ) - 1
  //                  (  4096 x rate )
  const uint8_t prescale = std::round(OSC_CLOCK_HZ
                                      / (static_cast<float>(MAX_VALUE)
                                         * static_cast<float>(clampedFreqModHz)))
                           - 1;

  RCLCPP_DEBUG(logger_,
               "Setting PWM frequency to %d Hz (prescale: %d)",
               clampedFreqModHz,
               prescale);

  const uint8_t oldmode = read(REGISTER_MODE1);

  // PRE_SCALE register can only be updated when SLEEP mode enabled
  RCLCPP_DEBUG(logger_, "MODE1: Sleep");
  write(REGISTER_MODE1, (oldmode & CLEAR_RESTART) | SLEEP);

  RCLCPP_DEBUG(logger_, "PRE_SCALE");
  write(REGISTER_PRE_SCALE, prescale);

  // Restore the original mode, minus SLEEP
  RCLCPP_DEBUG(logger_, "MODE1: Restore");
  write(REGISTER_MODE1, oldmode);

  nanosleep(&OSC_STABILIZATION_DELAY, NULL);

  // restart
  RCLCPP_DEBUG(logger_, "MODE1: Restart...");
  write(REGISTER_MODE1, oldmode | RESTART);
}

uint8_t Pca9685Impl::read(uint8_t reg) const
{
  const int result = i2c_smbus_read_byte_data(fd_, reg);

  if (result < 0) {
    RCLCPP_ERROR(logger_,
                 "Unable to read I2C device register 0x%x; errno: %d",
                 reg,
                 result);

    return 0;
  } else {
    return static_cast<uint8_t>(result);
  }
}

void Pca9685Impl::write(uint8_t reg, uint8_t data)
{
  RCLCPP_DEBUG(logger_,
               "Writing 0x%02x <-- 0b%s",
               reg,
               std::bitset<8>(data).to_string().c_str());

  const int result = i2c_smbus_write_byte_data(fd_, reg, data);

  if (result < 0) {
    RCLCPP_ERROR(logger_,
                 "Unable to write I2C device register 0x%x; errno: %d",
                 reg,
                 result);
  }
}

void Pca9685Impl::writeChannel(uint8_t channel, uint16_t offValue)
{
  writeChannel(channel, 0, offValue);
}

void Pca9685Impl::writeChannel(uint8_t channel,
                               uint16_t onValue,
                               uint16_t offValue)
{
  if (channel > NUM_CHANNELS) {
    std::ostringstream ostr;
    ostr << "Channel out of range: " << channel;
    throw std::runtime_error(ostr.str());
  }

  if (offValue > MAX_VALUE) {
    std::ostringstream ostr;
    ostr << "Value out of range: " << offValue;
    throw std::runtime_error(ostr.str());
  }

  if (offValue < onValue) {
    std::ostringstream ostr;
    ostr << "Invalid onValue/offValue combination: "
         << onValue
         << ", "
         << offValue;
    throw std::runtime_error(ostr.str());
  }

  const int channelOffset = channel * 4;

  const uint16_t scaledOnValue = onValue * frequencyScale_;
  const uint16_t scaledOffValue = offValue * frequencyScale_;

  RCLCPP_DEBUG(logger_,
               "Channel %d ON: %d (%0.2fms)",
               REGISTER_CHANNEL0_ON_LOW + channelOffset,
               scaledOnValue,
               pulseWidthToMillis(scaledOnValue));
  RCLCPP_DEBUG(logger_,
               "Channel %d OFF: %d (%0.2fms)",
               REGISTER_CHANNEL0_OFF_LOW + channelOffset,
               scaledOffValue,
               pulseWidthToMillis(scaledOffValue));

  write(REGISTER_CHANNEL0_ON_LOW + channelOffset,
        scaledOnValue & LOWER_BYTE_MASK);
  write(REGISTER_CHANNEL0_ON_HIGH + channelOffset,
        scaledOnValue >> UPPER_BYTE_SHIFT);
  write(REGISTER_CHANNEL0_OFF_LOW + channelOffset,
        scaledOffValue & LOWER_BYTE_MASK);
  write(REGISTER_CHANNEL0_OFF_HIGH + channelOffset,
        scaledOffValue >> UPPER_BYTE_SHIFT);
}

float Pca9685Impl::pulseWidthToMillis(uint16_t pulseCounts) const
{
  // E.g. 50Hz == 0.002s == 20ms
  const float secondsPerCycle = 1.0f / frequencyHz_;
  const float msPerCycle = secondsPerCycle * 1000.0f;
  const float msPerCount = msPerCycle / static_cast<float>(Pca9685::MAX_VALUE);

  return pulseCounts * msPerCount;
}

}  // namespace i2c_pwm
