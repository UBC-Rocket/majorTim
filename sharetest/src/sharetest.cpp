#include <inttypes.h>

#include "mbed.h"
#include <sharetest.h>
#include <i2c_driver.h>

DigitalOut led1(LED1);

int main(void) 
{
    int32_t p = 0;
    int32_t t = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    if (barometerInit() != STATUS_OK) {
        printf("ERROR: barometer initialization failed :(\n");
    }

    while (1) {
        led1 = !led1;
        // wait(SHARETEST_INTERVAL);

        if (barometerGetCompensatedValues(&p, &t) != STATUS_OK) {
            printf("ERROR: barometer get compensated values failed :(\n");
        } else {
            printf("Pressure: %" PRIu32 " Temperature: %" PRIu32 "\n", p, t); 
        }

        if (accelerometerGetData(&x, &y, &z) != STATUS_OK) {
            printf("ERROR: accelerometer get data failed :(\n");
        } else {
            printf("X: %" PRId16 "Y: %" PRId16 "Z: %" PRId16 "\n", x, y, z); 
        }
    }
}