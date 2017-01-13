#include "mbed.h"

DigitalOut  my_led(LED1);
InterruptIn my_button(USER_BUTTON);
PwmOut      my_pwm(PB_14);

void pressed() {
    if (my_pwm.read() == 0.25) {
        my_pwm.write(0.75);
    }
    else {
        my_pwm.write(0.25);
    }
}

int main()
{
    // Set PWM
    my_pwm.period_ms(10);
    my_pwm.write(0.5);
    
    // Set button
    my_button.fall(&pressed);
    
    while (1) {
        my_led = !my_led;
        wait(0.5); // 500 ms
    }
}

