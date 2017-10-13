/*
   drv_ms4525.c : driver for MS4525 differential pressure sensor

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

// MS4525 address 0x28 for most common version
#define MS4525_ADDR   0x28
#define STATUS_MASK   0x3F

#define FILTERING4525_ADC_MIN        0.001   //
#define FILTERING4525_ADC_MAX        0.01 //
#define FILTERING4525_ADC_MIN_AT     10 // when abs(delta between ADC and current value) is less than MIN_AT , apply MIN
#define FILTERING4525_ADC_MAX_AT     100 // when abs(delta between ADC and current value) is more than MAX_AT , apply MAX (interpolation in between)

uint32_t polling_interval_ms = 20; // (ms)
uint32_t last_measurement_time_ms = 0;

static volatile float temp_measurement = 0.0f;
static volatile float raw_diff_pressure_Pa = 0.0f;


static inline float sign(float x)
{
  return (x > 0) - (x < 0);
}

bool ms4525_init(void)
{
  uint8_t buf[1];
  bool airspeed_present = false;
  airspeed_present |= i2cRead(MS4525_ADDR, 0xFF, 1, buf);
  return airspeed_present;
}


void ms4525_update()
{
  uint8_t buf[4];

  uint32_t now_ms = millis();
  if(now_ms > last_measurement_time_ms + polling_interval_ms)
  {
    last_measurement_time_ms = now_ms;
    i2cRead(MS4525_ADDR, 0xFF, 4, buf);

    uint8_t status = (buf[0] & 0xC0) >> 6;
    if(status == 0x00) // good data packet
    {
      int16_t raw_diff_pressure = 0x3FFF & ((buf[0] << 8) + buf[1]);
      int16_t raw_temp = ( 0xFFE0 & ((buf[2] << 8) + buf[3])) >> 5;

      // Convert to Pa and K
      raw_diff_pressure_Pa = -(((float)raw_diff_pressure - 1638.3f) / 6553.2f - 1.0f) * 6894.757;
      temp_measurement = (0.097703957f * raw_temp)  + 223.0; // K
    }
    else // stale data packet - ignore
    {
      return;
    }
  }
}

void ms4525_read(float *diff_press, float *temperature)
{
  (*temperature) = temp_measurement;
  (*diff_press) = raw_diff_pressure_Pa;
}

//=================================================
// Asynchronus data storage
static uint8_t buf[4] = {0, 0, 0, 0};
static volatile uint8_t read_status;
static bool new_data = false;
static bool sensor_present = false;

void ms4525_read_CB(void)
{
  if (read_status != I2C_JOB_ERROR)
  {
    new_data = true;
    sensor_present = true;
  }
}

bool ms4525_present(void)
{
  return sensor_present;
}

void ms4525_async_read(float* diff_press, float* temperature)
{
  if (new_data)
  {
    uint8_t status = (buf[0] & 0xC0) >> 6;
    if(status == 0x00) // good data packet
    {
      int16_t raw_diff_pressure = 0x3FFF & ((buf[0] << 8) + buf[1]);
      int16_t raw_temp = ( 0xFFE0 & ((buf[2] << 8) + buf[3])) >> 5;
      // Convert to Pa and K
      raw_diff_pressure_Pa = -(((float)raw_diff_pressure - 1638.3f) / 6553.2f - 1.0f) * 6894.757;
      temp_measurement = (0.097703957f * raw_temp)  + 223.0; // K
    }
  }
  (*temperature) = temp_measurement;
  (*diff_press) = raw_diff_pressure_Pa;
}


void ms4525_async_update(void)
{
  static volatile uint32_t next_update_ms = 0;
  uint32_t now_ms = millis();

  // if it's not time to do anything, just return
  if (now_ms < next_update_ms)
  {
    return;
  }
  else
  {
    i2c_queue_job(READ, MS4525_ADDR, 0xFF, buf, 4, &read_status, &ms4525_read_CB);
    next_update_ms += 20; // Poll at 50 Hz
  }
  return;
}
