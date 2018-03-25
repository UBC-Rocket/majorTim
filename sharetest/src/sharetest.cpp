#include <inttypes.h>

#include "mbed.h"
#include <sharetest.h>
#include <i2c_driver.h>
#include <MS5607I2C.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"


DigitalOut led1(LED1);

// Instantiate the SDBlockDevice by specifying the SPI pins connected to the SDCard
// socket. The PINS are:
//     MOSI (Master Out Slave In)
//     MISO (Master In Slave Out)
//     SCLK (Serial Clock)
//     CS (Chip Select)
SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);

static const char* sdSensorDataPath = "/sd/sensorData.txt";
static const char* sdMountPt = "sd";
FATFileSystem fs(sdMountPt, &sd);

int main(void) {
    float p = 0;
    float t = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    MS5607I2C ms5607(I2C_SDA, I2C_SCL, true);

    while (barometerInit() != STATUS_OK) {
        printf("ERROR: barometer initialization failed :(\r\n");
        wait(SHARETEST_INTERVAL);
    }

    while (accelerometerInit() != STATUS_OK) {
        printf("ERROR: accelerometer initialization failed :(\r\n");
        wait(SHARETEST_INTERVAL);
    }

    //append sensor data
    FILE* f = fopen(sdSensorDataPath, "a");
    while (f == NULL) {
        printf("ERROR: sd initialization failed :(\r\n");
        wait(SHARETEST_INTERVAL);
        f = fopen(sdSensorDataPath, "a");
    }

    while (true) {
        led1 = !led1;
        wait(SHARETEST_INTERVAL);

        if (barometerGetCompensatedValues(&p, &t) != STATUS_OK) {
            printf("ERROR: barometer get compensated values failed :(\r\n");
        } else {
            printf("Pressure: %.2f Temperature: %.2f\r\n", p, t);
            fprintf(f, "p: %.2f t: %.2f\n", p, t); //log to sd
        }
        /*ALT DRIVER*/
        // printf("ALT DRIVER - ");
        // printf("Pressure:    %.0f\t", ms5607.getPressure());
        // printf("Temperature: %.2f\n", ms5607.getTemperature());
        /*END ALT DRIVER*/

        if (accelerometerGetData(&x, &y, &z) != STATUS_OK) {
            printf("ERROR: accelerometer get data failed :(\r\n");
        } else {
            printf("X: %" PRId16 " Y: %" PRId16 " Z: %" PRId16 "\r\n", x, y, z);
            fprintf(f, "X: %" PRId16 " Y: %" PRId16 " Z: %" PRId16 "\r\n", x, y, z); //log to sd
        }
    }
}