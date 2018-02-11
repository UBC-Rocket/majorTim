#include "mbed.h"
#include <sharetest.h>

DigitalOut led1(LED1);

int main(void) {
    while (1) {
       led1 = !led1;
       wait(SHARETEST_INTERVAL);
    }
}