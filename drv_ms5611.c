/*
   drv_ms5611.c : driver for Measurement Specialties MS5611 barometer

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_ms5611.c

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
#include <limits.h>

// MS5611, Standard address 0x77
#define MS5611_ADDR             0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t  ms5611_osr = CMD_ADC_4096;
float pressure = 0.0;
float temperature = 0.0;
static uint8_t init_state = 0;

static void ms5611_reset(void)
{
  i2cWrite(MS5611_ADDR, CMD_RESET, 1);
  delayMicroseconds(2800);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
  uint8_t rxbuf[2] = { 0, 0 };
  i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
  return rxbuf[0] << 8 | rxbuf[1];
}

int8_t ms5611_crc(uint16_t *prom)
{
  int32_t i, j;
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;

  bool blankEeprom = true;

  for (i = 0; i < 16; i++) {
    if (prom[i >> 1]) {
      blankEeprom = false;
    }
    if (i & 1)
      res ^= ((prom[i >> 1]) & 0x00FF);
    else
      res ^= (prom[i >> 1] >> 8);
    for (j = 8; j > 0; j--) {
      if (res & 0x8000)
        res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (!blankEeprom && crc == ((res >> 12) & 0xF))
    return 0;

  return -1;
}

static uint32_t ms5611_read_adc(void)
{
  uint8_t rxbuf[3];
  i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
  return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void)
{
  i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void)
{
  ms5611_ut = ms5611_read_adc();
}

static void ms5611_start_up(void)
{
  i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void)
{
  ms5611_up = ms5611_read_adc();
}

static void ms5611_calculate()
{
  int32_t press = 0;
  int64_t temp = 0;
  int64_t delt = 0;
  if(ms5611_up > 9085466 * 2 / 3 && ms5611_ut > 0)
  {
    int64_t dT = (int64_t)ms5611_ut - ((int64_t)ms5611_c[5] << 8);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
      delt = temp - 2000;
      delt = 5 * delt * delt;
      off -= delt >> 1;
      sens -= delt >> 2;
      if (temp < -1500) { // temperature lower than -15degC
        delt = temp + 1500;
        delt = delt * delt;
        off -= 7 * delt;
        sens -= (11 * delt) >> 1;
      }
      temp -= ((dT * dT) >> 31);
    }
    press = ((((uint64_t)ms5611_up * sens) >> 21) - off) >> 15;

    pressure = (float)press; // Pa
    temperature = (float)temp/ 100.0 + 273.0; // K

//    float pre_adjust_altitude = fast_alt(pressure);

//    altitude = fast_alt(pressure) - offset;
  }
}


// =======================================================================================

bool ms5611_init(void)
{
  bool ack = false;
  uint8_t sig;
  int i;

  while(millis() < 10); // No idea how long the chip takes to power-up, but let's make it 10ms

  ack = i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
  if (!ack)
    return false;
  else
    init_state = 1;

  ms5611_reset();

  init_state = 2;

  // read all coefficients
  for (i = 0; i < PROM_NB; i++)
    ms5611_c[i] = ms5611_prom(i);
  // check crc, bail out if wrong
  if (ms5611_crc(ms5611_c) != 0)
    return false;

  return true;
}

void ms5611_update(void)
{
  static uint32_t next_time_ms = 0;
  static int state = 0;

  if(millis() > next_time_ms)
  {
    if (state)
    {
      ms5611_get_up();
      ms5611_start_ut();
      next_time_ms += 10;
      state = 0;
    }
    else
    {
      ms5611_get_ut();
      ms5611_start_up();
      state = 1;
      next_time_ms += 10;
      ms5611_calculate();
    }
  }
}

void ms5611_read(float* press, float* temp)
{
  (*press) = pressure;
  (*temp) = temperature;
}


//===================================================================
// ASYNC FUNCTIONS
static volatile uint8_t pressure_buffer[3];
static volatile uint8_t temp_buffer[3];

static uint8_t temp_command = 1;
static uint8_t pressure_command = 1;
static volatile uint8_t temp_start_status = 0;
static volatile uint8_t temp_read_status = 0;
static volatile uint8_t pressure_read_status = 0;
static volatile uint8_t pressure_start_status = 0;
static volatile uint8_t baro_state = 0;
static volatile uint32_t next_update_ms = 0;

static volatile uint8_t init_status;
static uint8_t init_command;

void ms5611_init_CB(void)
{
  // we successfully found the sensor
  if (init_status == I2C_JOB_COMPLETE)
  {
    init_state ++;
    next_update_ms = millis() + 30;
  }
}

void temp_request_CB(void)
{
  next_update_ms = millis() + 15;
}

void pressure_request_CB(void)
{
  next_update_ms = millis() + 15;
}

void pressure_read_CB(void)
{
  ms5611_up = (pressure_buffer[0] << 16) | (pressure_buffer[1] << 8) | pressure_buffer[2];
  next_update_ms = millis() + 15;
}

static void temp_read_CB(void)
{
  ms5611_ut = (temp_buffer[0] << 16) | (temp_buffer[1] << 8) | temp_buffer[2];
  next_update_ms = millis() + 15;
}

void ms5611_async_update(void)
{
  uint32_t now_ms = millis();

  // if it's not time to do anything, just return
  if (now_ms < next_update_ms)
  {
    return;
  }
  else
  {
    // this may be reduced by an i2c callback, but at the very least, monitor at 10 Hz
    next_update_ms += 100;
  }

  // Try to initialize the sensor if we haven't already
//  if (init_state  < 2)
//  {
//    if (init_state == 0)
//    {
//      init_command = 1;
//      i2c_queue_job(READ, MS5611_ADDR, CMD_PROM_RD, &init_command, 1, &init_status, ms5611_init_CB);
//      return;
//    }
//    else if (init_state == 1)
//    {
//      init_command = 1;
//      i2c_queue_job(READ, MS5611_ADDR, CMD_RESET, &init_command, 1, &init_status, ms5611_init_CB);
//      return;
//    }
//  }

  switch (baro_state)
  {
  case 0:
    // Read the pressure
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  pressure_buffer,
                  3,
                  &pressure_read_status,
                  &pressure_read_CB);
    baro_state = 1;
    break;
  case 1:
    // start a temp conversion
    i2c_queue_job(WRITE,
                  MS5611_ADDR,
                  CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr,
                  &temp_command,
                  1,
                  &temp_start_status,
                  &temp_request_CB);
    baro_state = 2;
    break;
  case 2:
    // Read the temperature
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  temp_buffer,
                  3,
                  &temp_read_status,
                  &temp_read_CB);
    baro_state = 3;
    break;
  case 3:
    // start a pressure conversion
    i2c_queue_job(WRITE,
                  MS5611_ADDR,
                  CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr,
                  &pressure_command,
                  1,
                  &pressure_start_status,
                  &pressure_request_CB);
    baro_state = 0;
    break;
  default:
    baro_state = 0;
    break;
  }
  ms5611_calculate();
}

bool ms5611_present()
{
  return init_state > 0;
}

void ms5611_async_read(float* press, float* temp)
{
  (*press) = pressure;
  (*temp) = temperature;
}
