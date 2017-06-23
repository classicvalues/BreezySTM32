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
float altitude = 0.0;
float pressure = 0.0;
float temperature = 0.0;
float offset = 0.0;
static bool calibrated = false;

static const int32_t max_pressure = 106598;
static const int32_t min_pressure = 69681;
static const int16_t num_entries = 500;
static const int16_t dx = 73;
static const int16_t lookup_table[500] = {
  26882,	26809,	26735,	26662,	26589,	26516,	26443,	26370,	26297,	26224,
  26151,	26078,	26006,	25933,	25860,	25788,	25715,	25643,	25571,	25499,
  25426,	25354,	25282,	25210,	25138,	25066,	24994,	24923,	24851,	24779,
  24708,	24636,	24565,	24493,	24422,	24351,	24279,	24208,	24137,	24066,
  23995,	23924,	23853,	23782,	23712,	23641,	23570,	23500,	23429,	23359,
  23288,	23218,	23147,	23077,	23007,	22937,	22867,	22797,	22727,	22657,
  22587,	22517,	22447,	22378,	22308,	22239,	22169,	22100,	22030,	21961,
  21892,	21822,	21753,	21684,	21615,	21546,	21477,	21408,	21339,	21270,
  21202,	21133,	21064,	20996,	20927,	20859,	20790,	20722,	20654,	20585,
  20517,	20449,	20381,	20313,	20245,	20177,	20109,	20041,	19973,	19906,
  19838,	19770,	19703,	19635,	19568,	19500,	19433,	19366,	19298,	19231,
  19164,	19097,	19030,	18963,	18896,	18829,	18762,	18695,	18629,	18562,
  18495,	18429,	18362,	18296,	18229,	18163,	18096,	18030,	17964,	17898,
  17832,	17765,	17699,	17633,	17567,	17502,	17436,	17370,	17304,	17238,
  17173,	17107,	17042,	16976,	16911,	16845,	16780,	16715,	16649,	16584,
  16519,	16454,	16389,	16324,	16259,	16194,	16129,	16064,	15999,	15935,
  15870,	15805,	15741,	15676,	15612,	15547,	15483,	15419,	15354,	15290,
  15226,	15162,	15098,	15033,	14969,	14905,	14842,	14778,	14714,	14650,
  14586,	14523,	14459,	14395,	14332,	14268,	14205,	14141,	14078,	14015,
  13951,	13888,	13825,	13762,	13699,	13636,	13572,	13510,	13447,	13384,
  13321,	13258,	13195,	13133,	13070,	13007,	12945,	12882,	12820,	12757,
  12695,	12632,	12570,	12508,	12446,	12383,	12321,	12259,	12197,	12135,
  12073,	12011,	11949,	11888,	11826,	11764,	11702,	11641,	11579,	11517,
  11456,	11394,	11333,	11272,	11210,	11149,	11088,	11026,	10965,	10904,
  10843,	10782,	10721,	10660,	10599,	10538,	10477,	10416,	10355,	10295,
  10234,	10173,	10113,	10052,	9991,	9931,	9871,	9810,	9750,	9689,
  9629,	9569,	9509,	9448,	9388,	9328,	9268,	9208,	9148,	9088,
  9028,	8969,	8909,	8849,	8789,	8730,	8670,	8610,	8551,	8491,
  8432,	8372,	8313,	8253,	8194,	8135,	8076,	8016,	7957,	7898,
  7839,	7780,	7721,	7662,	7603,	7544,	7485,	7426,	7367,	7309,
  7250,	7191,	7133,	7074,	7015,	6957,	6898,	6840,	6782,	6723,
  6665,	6607,	6548,	6490,	6432,	6374,	6316,	6258,	6199,	6141,
  6084,	6026,	5968,	5910,	5852,	5794,	5737,	5679,	5621,	5563,
  5506,	5448,	5391,	5333,	5276,	5218,	5161,	5104,	5046,	4989,
  4932,	4875,	4818,	4760,	4703,	4646,	4589,	4532,	4475,	4418,
  4362,	4305,	4248,	4191,	4134,	4078,	4021,	3964,	3908,	3851,
  3795,	3738,	3682,	3625,	3569,	3513,	3456,	3400,	3344,	3288,
  3232,	3175,	3119,	3063,	3007,	2951,	2895,	2839,	2783,	2728,
  2672,	2616,	2560,	2504,	2449,	2393,	2337,	2282,	2226,	2171,
  2115,	2060,	2004,	1949,	1894,	1838,	1783,	1728,	1673,	1617,
  1562,	1507,	1452,	1397,	1342,	1287,	1232,	1177,	1122,	1067,
  1013,	958,	903,	848,	794,	739,	684,	630,	575,	521,
  466,	412,	357,	303,	248,	194,	140,	86,	31,	-23,
  -77,	-131,	-185,	-239,	-293,	-347,	-401,	-455,	-509,	-563,
  -617,	-671,	-725,	-778,	-832,	-886,	-940,	-993,	-1047,	-1100,
  -1154,	-1207,	-1261,	-1314,	-1368,	-1421,	-1475,	-1528,	-1581,	-1634,
  -1688,	-1741,	-1794,	-1847,	-1900,	-1953,	-2006,	-2060,	-2112,	-2165,
  -2218,	-2271,	-2324,	-2377,	-2430,	-2483,	-2535,	-2588,	-2641,	-2693,
  -2746,	-2799,	-2851,	-2904,	-2956,	-3009,	-3061,	-3114,	-3166,	-3218,
  -3271,	-3323,	-3375,	-3428,	-3480,	-3532,	-3584,	-3636,	-3688,	-3740,
};

static float fast_alt(float press)
{

  if(press < max_pressure && press > min_pressure)
  {
    float t = (float)num_entries*(press - (float)min_pressure) / (float)(max_pressure - min_pressure);
    int16_t index = (uint16_t)t;
    float dp = t - (float)index;

    float out = 0.0;

    if (index < num_entries - 1)
    {
      out = lookup_table[index]/10.0 + dp * (lookup_table[index + 1] - lookup_table[index])/10.0;
    }
    else
    {
      out = lookup_table[index]/10.0 + dp * (lookup_table[index] - lookup_table[index - 1])/10.0;
    }

    return out;
  }
  else
    return 0.0;
}

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

static void ms5611_calibrate(float pre_adjust_altitude)
{
  static uint16_t calibration_counter = 0;
  static float calibration_sum = 0.0f;

  calibration_counter++;
  if(calibration_counter == 256)
  {
    offset = calibration_sum / 127.0;
    calibration_counter = 0;
    calibration_sum = 0.0f;
    calibrated = true;
  }
  else if(calibration_counter > 128)
  {
    calibration_sum += pre_adjust_altitude;
  }
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

    float pre_adjust_altitude = fast_alt(pressure);

    if(!calibrated && pressure)
      ms5611_calibrate(pre_adjust_altitude);

    altitude = fast_alt(pressure) - offset;
  }
}


// =======================================================================================

void ms5611_start_calibration(void)
{
  calibrated = false;
}

bool ms5611_init(void)
{
  bool ack = false;
  uint8_t sig;
  int i;

  while(millis() < 10); // No idea how long the chip takes to power-up, but let's make it 10ms

  ack = i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
  if (!ack)
    return false;

  ms5611_reset();

  // read all coefficients
  for (i = 0; i < PROM_NB; i++)
    ms5611_c[i] = ms5611_prom(i);
  // check crc, bail out if wrong
  if (ms5611_crc(ms5611_c) != 0)
    return false;

  calibrated = false;

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

void ms5611_read(float* alt, float* press, float* temp)
{
  (*alt) = altitude;
  (*press) = pressure;
  (*temp) = temperature;
}


//===================================================================
// ASYNC FUNCTIONS
static uint8_t pressure_buffer[3];
static uint8_t temp_buffer[3];

static uint8_t temp_command = 1;
static uint8_t pressure_command = 1;
static volatile uint8_t temp_start_status = 0;
static volatile uint8_t temp_read_status = 0;
static volatile uint8_t pressure_read_status = 0;
static volatile uint8_t pressure_start_status = 0;
static uint8_t baro_state = 0;
static uint32_t next_update_ms = 0;
static uint32_t baro_last_update_watchdog_ms = 0;

void temp_request_CB(void)
{
  next_update_ms = millis() + 20;
}

void pressure_request_CB(void)
{
  next_update_ms = millis() + 20;
}

void pressure_read_CB(void)
{
  ms5611_up = (pressure_buffer[0] << 16) | (pressure_buffer[1] << 8) | pressure_buffer[2];
  next_update_ms = millis();
}

static void temp_read_CB(void)
{
  ms5611_ut = (temp_buffer[0] << 16) | (temp_buffer[1] << 8) | temp_buffer[2];
  next_update_ms = millis();
}

void ms5611_async_update(void)
{
  uint32_t now_ms = millis();

  // if it's not time to do anything, just return
  if (now_ms < next_update_ms)
  {
    return;
  }

  baro_last_update_watchdog_ms = now_ms;

  switch (baro_state)
  {
  case 0:
    // start a pressure read
    i2c_queue_job(WRITE,
                  MS5611_ADDR,
                  CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr,
                  &pressure_command,
                  1,
                  &pressure_start_status,
                  &pressure_request_CB);
    baro_state++;
    break;
  case 1:
    // Read the pressure
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  pressure_buffer,
                  3,
                  &pressure_read_status,
                  &pressure_read_CB);
    baro_state++;
    break;
  case 2:
    // start a temp read
    i2c_queue_job(WRITE,
                  MS5611_ADDR,
                  CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr,
                  &temp_command,
                  1,
                  &temp_start_status,
                  &temp_request_CB);
    baro_state++;
    break;
  case 3:
    // Read the temperature
    i2c_queue_job(READ,
                  MS5611_ADDR,
                  CMD_ADC_READ,
                  temp_buffer,
                  3,
                  &temp_read_status,
                  &temp_read_CB);
    baro_state = 0;
    break;
  }

  next_update_ms = millis() + 50;

  ms5611_calculate();
}

bool ms5611_present()
{
  if(ms5611_up != 0 || ms5611_ut != 0)
    return true;
  else
    return false;
}

void ms5611_async_read(float* alt, float* press, float* temp)
{
  (*alt) = altitude;
  (*press) = pressure;
  (*temp) = temperature;
}
