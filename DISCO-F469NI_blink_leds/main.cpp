#include "mbed.h"

DigitalOut led_green(LED1);
DigitalOut led_orange(LED2);
DigitalOut led_red(LED3);
DigitalOut led_blue(LED4);
DigitalOut LED_test(D7);
int main() {  
    while(1) {
        // WARNING: LEDs are OFF
        led_green = 1;
        led_orange = 1;
        led_red = 1;
        led_blue = 1;
				LED_test = 1;
        wait(0.2); // 200 ms
        // WARNING: LEDs are ON
        led_green = 0;
        led_orange = 0;
        led_red = 0;
        led_blue = 0;
				LED_test = 0;
        wait(1.0); // 1 sec
    }
}
