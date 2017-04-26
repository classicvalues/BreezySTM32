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

static uint8_t read_buffer[2] = {0,0};
static uint64_t last_update_time_us;
static uint64_t distance;



bool sen13680_init()
{
  // check for device SEN13680_DEFAULT_ADDRESS and set
  // to read fast and noisy if it's there
  return i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x04, 0x00);
}


void sen13680_update()
{
  // get current time
  uint64_t now_us = micros();

  // populate new measurement at 100hz
  if (now_us > last_update_time_us + 10000)
  {
    // save current time and wipe read_buffer
    last_update_time_us = now_us;
    read_buffer[0] = 0;
    read_buffer[1] = 0;

    // Request and read a lidar measurement
    i2cWrite(SEN13680_DEFAULT_ADDRESS, 0x00, 0x04);
    i2cRead(SEN13680_DEFAULT_ADDRESS, 0x8F, 2, read_buffer);

    // convert to cm
    distance = (read_buffer[0] << 8) + read_buffer[1];
  }
}

float sen13680_read()
{
  return (float)distance;
}
