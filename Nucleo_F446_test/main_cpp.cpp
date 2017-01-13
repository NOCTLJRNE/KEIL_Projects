#include "mbed.h"
#include "wait_api.c"
#include "us_ticker_api.c"
//DigitalOut myled(LED1);

int main() {
  test_var_wait_api = 1;  
	while(1) {
        //myled = 1; // LED is ON
        wait(0.2); // 200 ms
        //myled = 0; // LED is OFF
        //wait(1.0); // 1 sec
    }
		return 0;
}