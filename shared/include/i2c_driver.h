/* I2C Driver Header*/

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "general.h"

/* Constants -------------------------------------------------- */
#define BAROMETER_ADDRESS       0xEC
#define ACCELEROMETER_ADDRESS   0x30

#define BAROMETER_CMD_RESET     0x1E
#define BAROMETER_CMD_PROM_READ 0xA0
#define BAROMETER_CMD_ADC_READ  0x00
#define BAROMETER_CMD_ADC_CONV  0x40
#define BAROMETER_CMD_ADC_D1    0x00
#define BAROMETER_CMD_ADC_D2    0x10
#define BAROMETER_CMD_ADC_256   0x00
#define BAROMETER_CMD_ADC_512   0x02
#define BAROMETER_CMD_ADC_1024  0x04
#define BAROMETER_CMD_ADC_2048  0x06
#define BAROMETER_CMD_ADC_4096  0x08

#define ACCELEROMETER_AUTO_INCREMENT        0x80
#define ACCELEROMETER_REG_CTRL_REG1         0x20
#define ACCELEROMETER_REG_CTRL_REG2         0x21
#define ACCELEROMETER_REG_CTRL_REG3         0x22
#define ACCELEROMETER_REG_CTRL_REG4         0x23
#define ACCELEROMETER_REG_CTRL_REG5         0x24
#define ACCELEROMETER_REG_HP_FILTER_RESET   0x25
#define ACCELEROMETER_REG_REFERENCE         0x26
#define ACCELEROMETER_REG_STATUS_REG        0x27
#define ACCELEROMETER_REG_OUT_X_L           0x28
#define ACCELEROMETER_REG_OUT_X_H           0x29
#define ACCELEROMETER_REG_OUT_Y_L           0x2A
#define ACCELEROMETER_REG_OUT_Y_H           0x2B
#define ACCELEROMETER_REG_OUT_Z_L           0x2C
#define ACCELEROMETER_REG_OUT_Z_H           0x2D
#define ACCELEROMETER_REG_INT1_CFG          0x30
#define ACCELEROMETER_REG_INT1_SOURCE       0x31
#define ACCELEROMETER_REG_INT1_THS          0x32
#define ACCELEROMETER_REG_INT1_DURATION     0x33
#define ACCELEROMETER_REG_INT2_CFG          0x34
#define ACCELEROMETER_REG_INT2_SOURCE       0x35
#define ACCELEROMETER_REG_INT2_THS          0x36
#define ACCELEROMETER_REG_INT2_DURATION     0x37

/* Functions -------------------------------------------------- */
extern status_t i2cRead(uint16_t address, uint8_t *data, uint16_t size);
extern status_t i2cWrite(uint16_t address, uint8_t *data, uint16_t size);
extern status_t barometerReset(void);
extern status_t barometerGetCalibration(void);
extern status_t barometerInit(void);
extern status_t barometerGetData(uint8_t d, uint8_t osr, uint8_t *buffer);
extern status_t barometerGetUncompensatedValues(uint32_t *d1, uint32_t *d2);
extern status_t barometerCompensateValues(uint32_t d1, uint32_t d2, float *pressure, float *temperature);
extern status_t barometerGetCompensatedValues(float *pressure, float *temperature);
extern status_t barometerGetCompensatedPressure(float *pressure);
extern status_t accelerometerReadRegister(uint8_t sub_address, uint8_t *buffer, uint16_t size);
extern status_t accelerometerWriteRegister(uint8_t sub_address, uint8_t data);
extern status_t accelerometerInit(void);
extern status_t accelerometerGetData(int16_t *x, int16_t *y, int16_t *z);

#endif /* I2C_DRIVER_H */
