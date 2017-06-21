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

static float fastInvSqrt(float x);
inline static float absf(float x);

uint32_t polling_interval_ms = 20; // (ms)
uint32_t last_measurement_time_ms = 0;

static float temp = 0.0f;
static float raw_diff_pressure_Pa = 0.0f;
static float velocity = 0.0f;
static float diff_pressure_abs_Pa = 0.0f;
static float diff_pressure_smooth_Pa = 0.0f;
static float atmospheric_pressure = 101325.0f; // For ground level, should use ms5611 to provide

static bool calibrated = true;
static volatile float diff_pressure_offset = 0.0f;

static inline float sign(float x)
{
  return (x > 0) - (x < 0);
}

void ms4525_start_calibration()
{
  calibrated = false;
}

static void calibrate()
{
  static uint16_t calibrate_count = 0;
  static float calibration_sum = 0;

  calibrate_count++ ;
  if (calibrate_count == 256 )
  {
    diff_pressure_offset =  calibration_sum / 128.0f; //there has been 128 reading (256-128)
    calibrated = true;
    calibration_sum = 0.0f;
    calibrate_count = 0;
  }
  else if  (calibrate_count >= 128  )
  {
    // Let the sensor settle for the first 128 measurements
    calibration_sum += raw_diff_pressure_Pa;
  }
}

bool ms4525_calibrated()
{
  return calibrated;
}



bool ms4525_init(void)
{
  uint8_t buf[1];
  bool airspeed_present = false;
  airspeed_present |= i2cRead(MS4525_ADDR, 0xFF, 1, buf);
  calibrated = false;
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
      temp = (0.097703957f * raw_temp)  + 223.0; // K

      // Filter diff pressure measurement
      float LPF_alpha = 0.1;
      diff_pressure_abs_Pa = raw_diff_pressure_Pa - diff_pressure_offset;
      diff_pressure_smooth_Pa = diff_pressure_abs_Pa - LPF_alpha * (diff_pressure_abs_Pa - diff_pressure_smooth_Pa);

      velocity = sign(diff_pressure_smooth_Pa) * 24.574f/fastInvSqrt((absf(diff_pressure_smooth_Pa) * temp  /  atmospheric_pressure));

    }
    else // stale data packet - ignore
    {
      return;
    }

    if(!calibrated)
      calibrate();
  }

}

void ms4525_read(float *diff_press, float *temperature, float* vel)
{
  (*temperature) = temp;
  (*diff_press) = diff_pressure_smooth_Pa;
  (*vel) = velocity;
}

void ms4525_set_atm(uint32_t pressure_Pa)
{
  atmospheric_pressure = pressure_Pa;
}

//=================================================
// Asynchronus data storage
static uint8_t buf[4];
static volatile uint8_t read_status;
static volatile int16_t velocity_data;
static volatile int16_t temperature_data;

void ms4525_read_CB(void)
{
  int16_t data[2];
  uint8_t status = (buf[0] >> 5); // first two bits are status bits
  if(status == 0x00) // good data packet
  {
    data[0] = (int16_t)(((STATUS_MASK | buf[0]) << 8) | buf[1]);
    data[1] = (int16_t)((buf[2] << 3) | (buf[3] >> 5));
  }
  else if(status == 0x02) // stale data packet
  {
    data[0] = (int16_t)(((STATUS_MASK | buf[0]) << 8) | buf[1]);
    data[1] = (int16_t)((buf[2] << 3) | (buf[3] >> 5));
  }
  else
  {
    return;
  }
  velocity_data = data[0];
  temperature_data = data[1];
}

int16_t ms4525_read_velocity(void)
{
  return velocity_data;
}


int16_t ms4525_read_temperature(void)
{
  return temperature_data;
}


void ms4525_request_async_update(void)
{
  static uint64_t next_update_us = 0;
  uint64_t now_us = micros();

  // if it's not time to do anything, just return
  if((int64_t)(now_us - next_update_us) < 0)
  {
    return;
  }
  else
  {
    i2c_queue_job(READ, MS4525_ADDR, 0xFF, buf, 4, &read_status, &ms4525_read_CB);
    next_update_us = now_us + 1000; // Response time is 1 ms (1000 microseconds)
  }
  return;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

static float fastInvSqrt(float x)
{
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = x * 0.5F;
  y  = x;
  i  = * (long *) &y;                         // evil floating point bit level hacking
  i  = 0x5f3759df - (i >> 1);
  y  = * (float *) &i;
  y  = y * (threehalfs - (x2 * y * y));

  return y;
}

#pragma GCC diagnostic pop

inline static float absf(float x)
{
  return (x < 0) ? -x : x;
}
