/*
   drv_qmc5883l.h :  Support for QMC5883L Magnetometer
   a knock off of the hcm5883l magnetometer

   Author - James Jackson

   MIT license
 */

#include "drv_i2c.h"

bool qmc5883lInit();

// Blocking I2C Read Method
void qmc5883l_update();
void qmc5883l_read(int16_t *magData);

// Asynchronous I2C method
void qmc5883l_request_async_update();
void qmc5883l_async_read(int16_t *magData);
