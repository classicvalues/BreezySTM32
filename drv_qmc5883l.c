#include "drv_qmc5883l.h"
#include "drv_system.h"

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
  success = i2cWrite(ADDRESS, 0x09, MODE_CONTINUOUS | ODR_100Hz | RNG_2G | OSR_512);
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

/* =================================================================
 * Asynchronous Method
 */
static uint8_t mag_buffer[6];
static int16_t mag_data[3];
static volatile uint8_t status;

void mag_read_CB(void)
{
  mag_data[0] = (mag_buffer[0] | (mag_buffer[1] << 8));
  mag_data[1] = (mag_buffer[2] | (mag_buffer[3] << 8));
  mag_data[2] = (mag_buffer[4] | (mag_buffer[5] << 8));
}

void qmc5883l_request_async_update()
{
  static uint64_t last_update_ms = 0;
  uint64_t now = millis();

  if(now - last_update_ms > 7)
  {
    // 100 Hz update rate
    i2c_queue_job(READ,
                  ADDRESS,
                  0x00,
                  mag_buffer,
                  6,
                  &status,
                  &mag_read_CB);

    last_update_ms = now;
  }
  return;
}

void qmc5883l_async_read(int16_t *magData)
{
  magData[0] = mag_data[0];
  magData[1] = mag_data[1];
  magData[2] = mag_data[2];
  return;
}
