/*
 * init.h
 *
 *  Created on: 17 авг. 2018 г.
 *      Author: tymbys
 */

#ifndef INIT_H_
#define INIT_H_

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"

#define USE_TIMEOUT       0

#define SLAVE_OWN_ADDRESS                       0x5A /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits ADD[1:7] */

/**
  * @brief Master Transfer Request Direction
  */
#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01



void Configure_I2C_Slave(void);
void Activate_I2C_Slave(void);
void Handle_I2C_Slave(void);


#endif /* INIT_H_ */
