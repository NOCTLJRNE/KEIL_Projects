#include "mbed.h"
extern "C" 
{
	#include "TMP35.h"
}
//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

//using namespace TMP35;

Serial pc(SERIAL_TX, SERIAL_RX);
 
DigitalOut myled(LED1);
DigitalOut VCC(A0); 	//	VCC for TMPM5
DigitalOut GND(A2); 	// GND for TMP35
AnalogIn TMP_voltage(A1); // output voltage of TMP35
 
int main() 
{
  float TMP_voltage_value;
	float TMP_temp_value;
	int i = 1;
  pc.printf("Hello World !\n");
	VCC = 1;
	GND = 0;
  while(1) 
	{ 
      wait(1);
			TMP_voltage_value = TMP_voltage.read();
			//TMP_voltage_value = TMP_voltage_value*3300;
			//TMP_temp_value = (TMP_voltage_value - 500)/10;
		TMP_temp_value = temperature(TMP_voltage_value);
		pc.printf("Runs since %d seconds. TMP35 temperature: %f \r\n", i++, TMP_temp_value);
      myled = !myled;
  }
	return 0;
}

