#define STM32F469xx 
#include "mbed.h"

DigitalOut led_green(LED1);
DigitalOut led_orange(LED2);
DigitalOut led_red(LED3);
DigitalOut led_blue(LED4);

int main() {  
    while(1) {
        // WARNING: LEDs are OFF
        led_green = 1;
        led_orange = 1;
        led_red = 1;
        led_blue = 1;
        wait(0.2); // 200 ms
        // WARNING: LEDs are ON
        led_green = 0;
        led_orange = 0;
        led_red = 0;
        led_blue = 0;
        wait(1.0); // 1 sec
    }
}
