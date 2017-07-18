#include "drv_qmc5883l.h"

#define ADDRESS 0x0D

#define MODE_CONTINUOUS 0x01

#define ODR_10Hz        0x00
#define ODR_50Hz        0x04
#define ODR_100Hz       0x08
#define ODR_200Hz       0x0C

#define RNG_2G          0x00
#define RNG_8G          0x10

#define OSR_512         0x00
#define OSR_256         0x40
#define OSR_128         0x80
#define OSR_64          0xC0

bool qmc5883lInit()
{
  bool success = false;
  success = i2cWrite(ADDRESS, 0x0B, 0x01);
  success = i2cWrite(ADDRESS, 0x09, MODE_CONTINUOUS | ODR_200Hz | RNG_8G | OSR_512);
  return success;
}

void qmc5883l_read(int16_t *magData)
{
  uint8_t buf[6];
  i2cRead(ADDRESS, 0x00, 6, buf);
  magData[0] = (int16_t)(buf[0] << 8 | buf[1]);
  magData[1] = (int16_t)(buf[2] << 8 | buf[3]);
  magData[2] = (int16_t)(buf[4] << 8 | buf[5]);
}

