#include <inttypes.h>

#include "mbed.h"
#include <sharetest.h>
#include <MS5607I2C.h>

DigitalOut led1(LED1);

int main(void) 
{
    /*int32_t p = 0;
    int32_t t = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    while (barometerInit() != STATUS_OK) {
        printf("ERROR: barometer initialization failed :(\r\n");
    }

    while (1) {
        led1 = !led1;
        // wait(SHARETEST_INTERVAL);

        if (barometerGetCompensatedValues(&p, &t) != STATUS_OK) {
            printf("ERROR: barometer get compensated values failed :(\r\n");
        } else {
            printf("Pressure: %" PRIu32 " Temperature: %" PRIu32 "\r\n", p, t); 
        }

        if (accelerometerGetData(&x, &y, &z) != STATUS_OK) {
            printf("ERROR: accelerometer get data failed :(\r\n");
        } else {
            printf("X: %" PRId16 "Y: %" PRId16 "Z: %" PRId16 "\r\n", x, y, z); 
        }
    }*/

    MS5607I2C ms5607(I2C_SDA, I2C_SCL, true);

    while (true) {
        printf("Pressure:    %.0f Pa\t",   ms5607.getPressure());
        printf("Temperature: %.2f degC\t", ms5607.getTemperature());
        printf("Altitude:    %.2f m\n",    ms5607.getAltitude());
    }



}