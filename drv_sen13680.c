/*
   drv_sen13680.c : driver for Sparkfun SEN13680 LIDAR-Lite v2

   Copyright (C) 2016 Simon D. Levy

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

// See https://github.com/PulsedLight3D for Arduino code
// and more documentation on this sensor

#include <breezystm32.h>

#include <stdbool.h>
#include <stdint.h>

#include "drv_i2c.h"

#define SEN13680_DEFAULT_ADDRESS 0x62

static int16_t distance;



bool sen13680_init()
{
  // check for device SEN13680_DEFAULT_ADDRESS and set
  // to read fast and noisy if it's there
  bool success = true;
  if (i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x00, 0x00))
  {
    // Set the time between measurements (0x45).  0x04 means 250 Hz
    success &= i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x45, 0x04);
    // Set the mode pin to default setting
    success &= i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x04, 0x21);
    // Set the number of measurements to be taken (continuous)
    success &= i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x11, 0xFF);
    // Initiate Reading
    success &= i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x00, 0x04);
  }
  else
  {
    success = false;
  }
  return success;
}


void sen13680_update()
{
  // get current time
  uint64_t now_us = micros();
  static uint64_t last_update_time_us = 0;

  // populate new measurement at 100hz
  if (now_us > last_update_time_us + 10000)
  {
    // save current time
    last_update_time_us = now_us;
    uint8_t read_buffer[2] = {100,100};

    // Request and read a lidar measurement
    i2cRead(SEN13680_DEFAULT_ADDRESS, 0x8F, 2, read_buffer);

    distance = (int16_t)(read_buffer[0] << 8) + read_buffer[1];
  }
}

float sen13680_read()
{
  return (float)distance;
}
