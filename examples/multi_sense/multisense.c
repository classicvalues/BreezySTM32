/*
   accelgyro.c : report accelerometer and gyroscope values

   Copyright (C) 2016 James Jackson

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

#define BOARD_REV 2

float accel_scale; // converts to units of m/s^2
float gyro_scale; // converts to units of rad/s

int16_t accel_data[3];
int16_t gyro_data[3];
volatile int16_t temp_data;
int16_t mag_data[3];

volatile uint8_t accel_status = 0;
volatile uint8_t gyro_status = 0;
volatile uint8_t temp_status = 0;
volatile bool mpu_new_measurement = false;

uint32_t start_time = 0;

bool baro_present= false;
bool mag_present=false;
bool sonar_present=false;
bool airspeed_present=false;
void setup(void)
{
  delay(500);
  i2cInit(I2CDEV_2);

  // Init Baro
  i2cWrite(0, 0, 0);
  ms5611_init();

  // Init Mag
  hmc5883lInit(BOARD_REV);

  // Init Sonar
  //  mb1242_init();

  // Init Airspeed
  //  airspeed_present = ms4525_detect();

  //Init IMU (has to come last because of the ISR)
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale, BOARD_REV);
  accel_scale = 9.80665f / acc1G;
}

void loop(void)
{

  float baro = 0;
  float temp = 0;
  //  int32_t sonar = 0;
  int32_t airspeed = 0;
  // Update Baro
  ms5611_async_update();
  if(ms5611_present())
  {
    ms5611_async_read(&baro, &temp);
  }

  // Update Mag
  hmc5883l_request_async_update();
  if(hmc5883l_present())
  {
    hmc5883l_async_read(mag_data);
  }

  //  // Update Sonar
  //  if(sonar_present)
  //  {
  //    sonar = mb1242_poll();
  //  }

  //  // Update Airspeed
  //  if(airspeed_present)
  //  {
  //    ms4525_request_async_update();
  //    airspeed = ms4525_read_velocity();
  //  }

  static uint64_t imu_timestamp_us = 0;
  if (mpu6050_new_data())
  {
    mpu6050_async_read_all(accel_data, &temp_data, gyro_data, &imu_timestamp_us);
  }

  static uint32_t last_print_ms = 0;
  // Throttle printing
  if(millis() > last_print_ms + 10)
  {
    last_print_ms += 10;
    printf("%d\t %d\t %d\t %d\t %d\t %d\n",
           (int32_t)(accel_data[2]*accel_scale*1000.0f),
        (int32_t)(gyro_data[2]*gyro_scale*1000.0f),
        (int32_t)imu_timestamp_us,
        (int32_t)mag_data[2],
        (int32_t)baro,
        (int32_t)i2cGetErrorCounter());
  }
}
