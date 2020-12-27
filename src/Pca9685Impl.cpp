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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

// Ref: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf

namespace {

const float OSC_CLOCK_HZ = 25000000.0;  // 25MHz
const uint16_t MAX_FREQ_MOD_HZ = 1526;
const uint16_t MIN_FREQ_MOD_HZ = 24;

const uint8_t REGISTER_MODE1 = 0x00;
const uint8_t REGISTER_MODE2 = 0x01;
const uint8_t REGISTER_CHANNEL0_ON_LOW = 0x06;
const uint8_t REGISTER_CHANNEL0_ON_HIGH = 0x07;
const uint8_t REGISTER_CHANNEL0_OFF_LOW = 0x08;
const uint8_t REGISTER_CHANNEL0_OFF_HIGH = 0x09;
const uint8_t REGISTER_PRE_SCALE = 0xFE;

const uint8_t RESTART = 0x80;
const uint8_t CLEAR_RESTART = 0x7F;
const uint8_t SLEEP = 0x10;

// By default, we only want ALLCALL to be enabled (bit 0):
const uint8_t DEFAULT_MODE1_MASK = 0x01;

// By default, we only want OUTDRV (totem) to be enabled (bit 2);
const uint8_t DEFAULT_MODE2_MASK = 0x04;

// Documentation states "at least" 500us
const struct timespec OSC_STABILIZATION_DELAY = { 0, 6000000L };

}  // namespace

namespace i2c_pwm {

Pca9685Impl::Pca9685Impl(const std::string &deviceFile, const int address)
  : fd_(::open(deviceFile.c_str(), O_RDWR)),
    address_(address),
    logger_(rclcpp::get_logger("i2c_pwm.Pca9685"))
{
  if (fd_ < 0) {
    std::ostringstream ostr;
    ostr << "Unable to open I2C device file: " << deviceFile;
    throw std::runtime_error(ostr.str());
  }

  Pca9685Impl::write(REGISTER_MODE1, DEFAULT_MODE1_MASK);
  Pca9685Impl::write(REGISTER_MODE2, DEFAULT_MODE2_MASK);

  // Initialize all servo channels to 0
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    Pca9685Impl::writeChannel(i, 0);
  }
}

Pca9685Impl::~Pca9685Impl()
{
  ::close(fd_);
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
  write(REGISTER_MODE1, (oldmode & CLEAR_RESTART) | SLEEP);
  write(REGISTER_PRE_SCALE, prescale);

  // Restore the original mode, minus SLEEP
  write(REGISTER_MODE1, oldmode);

  nanosleep(&OSC_STABILIZATION_DELAY, NULL);

  // restart
  write(REGISTER_MODE1, oldmode | RESTART);
}

uint8_t Pca9685Impl::read(uint8_t reg) const
{
  uint8_t value = 0;
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data msgset[1];

  msgs[0].addr = address_;
  msgs[0].flags = 0;
  msgs[0].len = 1;
  msgs[0].buf = &reg;

  msgs[1].addr = address_;
  msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
  msgs[1].len = 1;
  msgs[1].buf = &value;

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = 2;

  const int result = ::ioctl(fd_, I2C_RDWR, &msgset);
  if (result < 0) {
    std::ostringstream ostr;
    ostr << "Unable to read I2C device register 0x"
         << std::hex
         << reg
         << "; errno: "
         << std::dec
         << result;
    throw std::runtime_error(ostr.str());
  }

  return value;
}

void Pca9685Impl::write(uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = { reg, data };

  struct i2c_msg msgs[1];
  struct i2c_rdwr_ioctl_data msgset[1];

  msgs[0].addr = address_;
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = buf;

  msgset[0].msgs = msgs;
  msgset[0].nmsgs = 1;

  const int result = ::ioctl(fd_, I2C_RDWR, &msgset);
  if (result < 0) {
    std::ostringstream ostr;
    ostr << "Unable to write I2C device register 0x"
         << std::hex
         << reg
         << "; errno: "
         << std::dec
         << result;
    throw std::runtime_error(ostr.str());
  }
}

void Pca9685Impl::writeChannel(uint8_t channel, uint16_t value)
{
  if (channel > NUM_CHANNELS) {
    std::ostringstream ostr;
    ostr << "Channel out of range: " << channel;
    throw std::runtime_error(ostr.str());
  }

  if (value > MAX_VALUE) {
    std::ostringstream ostr;
    ostr << "Value out of range: " << value;
    throw std::runtime_error(ostr.str());
  }

  const int channelOffset = channel * 4;

  write(REGISTER_CHANNEL0_ON_LOW + channelOffset, 0);
  write(REGISTER_CHANNEL0_ON_HIGH + channelOffset, 0);
  write(REGISTER_CHANNEL0_OFF_LOW + channelOffset, value & 0xFF);
  write(REGISTER_CHANNEL0_OFF_HIGH + channelOffset, value >> 8);
}

}  // namespace i2c_pwm
