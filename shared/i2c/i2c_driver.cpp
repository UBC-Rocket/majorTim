/* I2C Driver Source*/

#include <math.h>
#include <stdint.h>

#include "mbed.h"
#include "I2C.h"
#include <i2c_driver.h>
#include <general.h>

/* Objects -------------------------------------------------- */
I2C i2c(I2C_SDA, I2C_SCL);

/* Variables -------------------------------------------------- */
static uint16_t barometer_calibration[6];

/* I2C Functions -------------------------------------------------- */

/**
  * @brief  Reads data from the I2C bus
  * @note   This is a single master system so repeated start conditions have been disabled
  * @param  address The address of the device to read from
  * @param  data A pointer to the buffer to write the data to
  * @param  size The number of bytes to read
  * @retval Status
  */
extern status_t i2cRead(uint16_t address, uint8_t *data, uint16_t size)
{
    if (i2c.read((int)address, (char*)data, (int)size, false) != 0) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
  * @brief  Writes data to the I2C bus
  * @note   This is a single master system so repeated start conditions have been disabled
  * @param  address The address of the device to write to
  * @param  data A pointer to the buffer to read the data from
  * @param  size The number of bytes to write
  * @retval Status
  */
extern status_t i2cWrite(uint16_t address, uint8_t *data, uint16_t size)
{
    if (i2c.write((int)address, (char*)data, (int)size, false) != 0) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/* Barometer Functions -------------------------------------------------- */

/**
  * @brief  Sends the reset command to the barometer
  * @note   This function should be called once during barometer initialization
  * @retval Status
  */
extern status_t barometerReset(void)
{
    uint8_t cmd = BAROMETER_CMD_RESET;

    if (i2cWrite(BAROMETER_ADDRESS, &cmd, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }
    /*need to wait for reset sequence to complete*/
    wait_ms(3);

    return STATUS_OK;
}

/**
  * @brief  Gets the calibration coefficients from the barometer's PROM
  * @note   This function should be called once during barometer initialization (must call barometerReset first!)
  *         Values are stored in the global array 'barometer_calibration'
  * @retval Status
  */
extern status_t barometerGetCalibration(void)
{
    uint8_t cmd;
    uint8_t buffer[2];

    for (int i = 0; i < 6; i++) {
        cmd = BAROMETER_CMD_PROM_READ | (i+1) * 2;
        if (i2cWrite(BAROMETER_ADDRESS, &cmd, 1) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (i2cRead(BAROMETER_ADDRESS, buffer, 2) != STATUS_OK) {
            return STATUS_ERROR;
        }
        barometer_calibration[i] = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }

    return STATUS_OK;
}

/**
  * @brief  Initializes the barometer
  * @note   This function should be called once during system initialization
  * @retval Status
  */
extern status_t barometerInit(void)
{
    if (barometerReset() != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (barometerGetCalibration() != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
  * @brief  Requests ADC conversion and then reads the resulting data
  * @param  d The requested data (choose from constants defined at top of file)
  * @param  osr The over sampling rate for the ADC conversion (choose from constants defined at top of file)
  * @param  buffer A pointer to the buffer to store the data
  * @retval Status
  */
extern status_t barometerGetData(uint8_t d, uint8_t osr, uint8_t *buffer)
{
    uint8_t cmd = BAROMETER_CMD_ADC_CONV | d | osr;

    if (i2cWrite(BAROMETER_ADDRESS, &cmd, 1)  != STATUS_OK) {
        return STATUS_ERROR;
    }
    /* Delay as needed to allow ADC conversion to complete */
    switch (osr & 0x0f) {
        case BAROMETER_CMD_ADC_256:
            wait_ms(1);
            break;
        case BAROMETER_CMD_ADC_512:
            wait_ms(2);
            break;
        case BAROMETER_CMD_ADC_1024:
            wait_ms(3);
            break;
        case BAROMETER_CMD_ADC_2048:
            wait_ms(5);
            break;
        case BAROMETER_CMD_ADC_4096:
            wait_ms(10);
            break;
        default:
            return STATUS_ERROR;
    }
    cmd = BAROMETER_CMD_ADC_READ;
    if (i2cWrite(BAROMETER_ADDRESS, &cmd, 1)  != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (i2cRead(BAROMETER_ADDRESS, buffer, 3) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
  * @brief  Gets the uncompensated pressure and temperature values
  * @param  d1 A pointer to store uncompensated pressure value in mbar
  * @param  d2 A pointer to store uncompensated temperature value in *C
  * @retval Status
  */
extern status_t barometerGetUncompensatedValues(uint32_t *d1, uint32_t *d2)
{
    uint8_t buffer[3];

    if (barometerGetData(BAROMETER_CMD_ADC_D1, BAROMETER_CMD_ADC_4096, buffer) != STATUS_OK) {
        return STATUS_ERROR;
    }
    *d1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];

    if (barometerGetData(BAROMETER_CMD_ADC_D2, BAROMETER_CMD_ADC_4096, buffer) != STATUS_OK) {
        return STATUS_ERROR;
    }
    *d2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];

    return STATUS_OK;
}

/**
  * @brief  Calculates the compensated pressure and temperature values
  * @note   The below algorithm is based on the MS5607-02BA03 datasheet
  * @param  d1 The uncompensated pressure value
  * @param  d2 The uncompensated temperature value
  * @param  pressure A pointer to store compensated pressure value
  * @param  temperature A pointer to store compensated temperature value
  * @retval Status
  */
extern status_t barometerCompensateValues(uint32_t d1, uint32_t d2, int32_t *pressure, int32_t *temperature)
{
    int32_t dt, p, t;
    int64_t off, sens;
    int32_t t2 = 0;
    int64_t off2 = 0;
    int64_t sens2 = 0;

    dt = d2 - ((int32_t)barometer_calibration[4] << 8);
    t = 2000 + ((dt * (int32_t)barometer_calibration[5]) >> 23);
    off = ((int64_t)barometer_calibration[1] << 17) + ((int64_t)barometer_calibration[3] * dt >> 6);
    sens = ((int64_t)barometer_calibration[0] << 16) + ((int64_t)barometer_calibration[2] * dt >> 7);

    if (t < 2000) {
        t2 = (int64_t)dt * dt >> 31;
        off2 = (int64_t)61 * (t - 2000) * (t - 2000) >> 4;
        sens2 = (int64_t)2 * (t - 2000) * (t - 2000);

        if (t < -1500) {
            off2 += (int64_t)15 * (t + 1500) * (t + 1500);
            sens2 += (int64_t)8 * (t + 1500) * (t + 1500);
        }
    }
    
    t -= t2;
    off -= off2;
    sens -= sens2;

    p = ((((int64_t)d1 * sens >> 21) - off) >> 15);

    /*TODO: convert to mbar and C*/
    *pressure = p;
    *temperature = t;

    return STATUS_OK;
}

/**
  * @brief  Gets the compensated pressure and temperature values
  * @param  pressure A pointer to store compensated pressure value
  * @param  temperature A pointer to store compensated temperature value
  * @retval Status
  */
extern status_t barometerGetCompensatedValues(int32_t *pressure, int32_t *temperature)
{
    uint32_t d1, d2;

    if (barometerGetUncompensatedValues(&d1, &d2) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (barometerCompensateValues(d1, d2, pressure, temperature) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
  * @brief  Gets the compensated pressure value
  * @param  pressure A pointer to store compensated pressure value
  * @retval Status
  */
extern status_t barometerGetCompensatedPressure(int32_t *pressure)
{
    uint32_t d1, d2;
    int32_t t;

    if (barometerGetUncompensatedValues(&d1, &d2) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (barometerCompensateValues(d1, d2, pressure, &t) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/* Accelerometer Functions -------------------------------------------------- */

/**
  * @brief  Reads from an accelerometer register(s)
  * @param  sub_address The address of the register to read from (choose from constants defined at top of file)
  * @param  data A pointer to the buffer to store the data
  * @param  size The number of bytes to read (0 < size <= 8)
  * @note   If size > 1 the register address is automatically incremented for reading from consecutive registers
  * @retval Status
  */
extern status_t accelerometerReadRegister(uint8_t sub_address, uint8_t *buffer, uint16_t size)
{
    uint8_t reg;

    if (size > 1) {
        reg = sub_address | ACCELEROMETER_AUTO_INCREMENT;
    } else {
        reg = sub_address;
    }

    if (i2cWrite(ACCELEROMETER_ADDRESS, &reg, 1) != STATUS_OK) {
        return STATUS_ERROR;
    }  

    if (i2cRead(ACCELEROMETER_ADDRESS, buffer, size) != STATUS_OK) {
        return STATUS_ERROR;
    }    

    return STATUS_OK;
}

/**
  * @brief  Writes to an accelerometer register
  * @param  sub_address The address of the register to write to (choose from constants defined at top of file)
  * @param  data The data to write
  * @retval Status
  */
extern status_t accelerometerWriteRegister(uint8_t sub_address, uint8_t data)
{
    uint8_t cmd[2];

    cmd[0] = sub_address;
    cmd[1] = data;
    if (i2cWrite(ACCELEROMETER_ADDRESS, cmd, 2) != STATUS_OK) {
        return STATUS_ERROR;
    }    

    return STATUS_OK;
}

/**
  * @brief  Initializes the accelerometer
  * @note   This function should be called once during system initialization
  * @retval Status
  */
extern status_t accelerometerInit(void)
{
    return STATUS_OK; //TODO: configure registers if needed
}

/**
  * @brief  Gets the x, y, and z acceleration data from the accelerometer's registers
  * @param  x A pointer to store x-axis acceleration value
  * @param  y A pointer to store y-axis acceleration value
  * @param  z A pointer to store z-axis acceleration value
  * @retval Status
  */
extern status_t accelerometerGetData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];

    if (accelerometerReadRegister(ACCELEROMETER_REG_OUT_X_L, buffer, 6) != STATUS_OK) {
        return STATUS_ERROR;
    }

    *x = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    *y = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
    *z = (uint16_t)buffer[4] | ((uint16_t)buffer[5] << 8);

    return STATUS_OK;
}
