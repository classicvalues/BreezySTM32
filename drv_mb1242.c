/*
   drv_mb1242.c : driver for MaxBotix MB1242 sonar

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

#include <breezystm32.h>

#include <stdbool.h>
#include <stdint.h>

#include "drv_i2c.h"

#define MB1242_DEFAULT_ADDRESS 0x70

static uint8_t start_measurement_command = 0x51;
static uint8_t read_buffer[2] = {0,0};
static volatile uint8_t start_measurement_status;
static volatile uint8_t read_measurement_status;
static uint8_t state = 0;
static uint64_t last_update_time_us;
static int32_t distance_cm;


static void adjust_reading(void) {

  distance_cm = 1.071 * distance_cm + 3.103; // emprically determined
}


bool mb1242_init()
{
  return i2cWrite(MB1242_DEFAULT_ADDRESS, 0xFF, start_measurement_command);
}


void start_sonar_measurement_CB(void)
{
  // indicate a completed started measurement
  state = 1;
}


void mb1242_update()
{
  uint64_t now_us = micros();
  if (now_us > last_update_time_us + 25000)
  {
    last_update_time_us = now_us;
    if (state == 0)
    {
      // Start a sonar measurement,
      i2cWrite(MB1242_DEFAULT_ADDRESS, 0xFF, start_measurement_command);
      state = 1;
    }
    else if (state == 1) {
      read_buffer[0] = 0;
      read_buffer[1] = 0;
      // Read the sonar measurement
      i2cRead(MB1242_DEFAULT_ADDRESS, 0xFF, 2, read_buffer);

      // convert to cm
      distance_cm = (read_buffer[0] << 8) + read_buffer[1];
      if (distance_cm > 850)
      {
          distance_cm = 0;
      }
      state = 0;
    }
  }
}

float mb1242_read()
{
  return (1.071 * (float)distance_cm + 3.103)/100.0; // emprically determined
}
