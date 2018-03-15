#include <inttypes.h>

#include "mbed.h"
#include <sharetest.h>
#include <i2c_driver.h>
#include <MS5607I2C.h>
#include "SDBlockDevice.h"

DigitalOut led1(LED1);

// Instantiate the SDBlockDevice by specifying the SPI pins connected to the SDCard
// socket. The PINS are:
//     MOSI (Master Out Slave In)
//     MISO (Master In Slave Out)
//     SCLK (Serial Clock)
//     CS (Chip Select)
SDBlockDevice sd(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);
uint8_t block[512] = "Hello World!\n";

int sdTest()
{
	// call the SDBlockDevice instance initialisation method.
	if ( 0 != sd.init()) {
		printf("Init failed \n");
		return -1;
	}
	printf("sd size: %llu\n",         sd.size());
	printf("sd read size: %llu\n",    sd.get_read_size());
	printf("sd program size: %llu\n", sd.get_program_size());
	printf("sd erase size: %llu\n",   sd.get_erase_size());

	// set the frequency
	if ( 0 != sd.frequency(5000000)) {
		printf("Error setting frequency \n");
	}

	if ( 0 != sd.erase(0, sd.get_erase_size())) {
		printf("Error Erasing block \n");
	}

	// Write some the data block to the device
	if ( 0 == sd.program(block, 0, 512)) {

		uint8_t readBuf[512];
		// read the data block from the device
		if ( 0 == sd.read(readBuf, 0, 512)) {
			// print the contents of the block
			printf("%s", readBuf);
		}
	}

	// call the SDBlockDevice instance de-initialisation method.
	sd.deinit();
	return 0;
}

int main(void)
{
    float p = 0;
    float t = 0;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    sdTest();

    MS5607I2C ms5607(I2C_SDA, I2C_SCL, true);

    while (barometerInit() != STATUS_OK) {
        printf("ERROR: barometer initialization failed :(\r\n");
        wait(SHARETEST_INTERVAL);
    }

    while (accelerometerInit() != STATUS_OK) {
        printf("ERROR: accelerometer initialization failed :(\r\n");
        wait(SHARETEST_INTERVAL);
    }


	while (1) {
        led1 = !led1;
        wait(SHARETEST_INTERVAL);

        if (barometerGetCompensatedValues(&p, &t) != STATUS_OK) {
            printf("ERROR: barometer get compensated values failed :(\r\n");
        } else {
            printf("Pressure: %.2f Temperature: %.2f\r\n", p, t);
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
        }
    }
}