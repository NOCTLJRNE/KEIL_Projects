/************************************************************************************************

 Author        : PHAM HO Bao An-CEA

 Date          : 04 November 2015

 File          : 

 Hardware      : ADuC706x

 Description   : PrimoSamp Firmware v1.1.1
 				 	machine à 4 états			 	
					0-Etat initial. 
						0->1 appui long sur le bouton 1(Green-Red GR) >3 secs (jusqu'à ce que sa couleur change).
						0->2 appui court sur le bouton 2 (Green-Blue GB).
						0->3 appui long sur le bouton 2 (>3 secs, jusqu'à ce que sa couleur change). 
					1-Regeneration: ~120 secs, 190°C, retour à l'état 0 aubout de 120 secs.
					2-Sampling:	pompe de prélèvement on, retour à l'état 0 en réappuyant (court) sur le bouton 2. 
					3-Injection: ~15 seconds, max temperature ramp to 220°C, then stabilize around 250°C, return to state 0 after 15 secs.
					
					A partir de l'état 1, 2, 3, retour à l'état 0 immediatement en appuyant simultanément 2 boutons. 
				 06/11/2015
				 	using I2C interrupt
 
				 	timed loop by ADuC7060 timer.
				 09/11/2015
				 	adding pump
				 10/11/2015
				 	H/W change : PCF8574N output is now "sink" instead of "source",
					state 2 is now Sample, state 3 is now Inject.
				 18/11/2015 v1.0.4
				 	adding pre-concentrator temperature reading.
				 07/01/2016 v1.1.0
				 	improve F/W for PrimoSamp prototype:
					- change n° switch of pump
					- temperature reading
					- preconcentrator heating
					- mode choice: with or without labVIEW interface
				 15/01/2016	v1.1.1
				 	si cartouche n'est pas détectée => blink LED..
				 24/03/2016 v1.1.2
				 	correction: l'interruption générérée quand le bus I2C du ADuC7060 finit la transmission des données est lue sur le BIT8 du registre I2CSTA
				 25/03/2016	v1.1.3
				 	permet de basculer facilement entre 2 modes: autonome (avec interface labview) et avec interface labview, pour les versions précédentes, il faut éteindre le boitier avant de basculer entre les 2 modes  					 
				 29/03/2016	v1.1.4
					correction des comportements bizarre lors d'un appui "aléatoire"				 

				 06/11/2015
				 	using I2C interrupt
				 	state machine (4 states: 0-Initial/Stand By, 1-Reset/Regeneration, 2-Sampling, 3-Injection)
					0-Initial state and stand by state. 
						0->1 by pressing button 1(Green-Red GR) more than 3 secs (until its color changes).
						0->2 by pushing shortly button 2 (Green-Blue GB).
						0->3 by pressing button 2 more than 3 secs (until its color changes). 
					1-Regeneration: ~120 seconds, 190°C (not required to ramp up fast), return to state 0 after 120 secs.
					2-Sampling:	pump on, return to state 0 by pushing shortly button 2. 
					3-Injection: ~15 seconds, max temperature ramp to 220°C, then stabilize around 250°C, return to state 0 after 15 secs.
					
					from state 1, 2, 3, return to state 0 immediately by pushing simultaneously 2 buttons.  
				 	timed loop by ADuC7060 timer.
				 09/11/2015
				 	adding pump
				 10/11/2015
				 	H/W change : PCF8574N output is now "sink" instead of "source",
					state 2 is now Sample, state 3 is now Inject.
				 18/11/2015 v1.0.4
				 	adding pre-concentrator temperature reading.
				 07/01/2016 v1.1.0
				 	improve F/W for PrimoSamp prototype:
					- change n° switch of pump
					- temperature reading
					- preconcentrator heating
					- mode choice: with or without labVIEW interface
				 15/01/2016
				 	error state added: if cartridge not detected => blink LED.. 	   	 		
*************************************************************************************************/
// Bit Definitions
#define BIT0  0x01
#define BIT1  0x02
#define BIT2  0x04
#define BIT3  0x08
#define BIT4  0x10
#define BIT5  0x20
#define BIT6  0x40
#define BIT7  0x80
#define BIT8  0x100
#define BIT9  0x200
#define BIT10 0x400
#define BIT11 0x800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000
#define BIT16 0x10000
#define BIT17 0x00020000
#define BIT18 0x00040000
#define BIT19 0x00080000
#define BIT20 0x00100000
#define BIT21 0x00200000
#define BIT22 0x00400000
#define BIT23 0x00800000
#define BIT24 0x01000000
#define BIT25 0x02000000
#define BIT26 0x04000000
#define BIT27 0x08000000
#define BIT28 0x10000000
#define BIT29 0x20000000
#define BIT30 0x40000000
#define BIT31 0x80000000
#define I2C_RD_BIT 0x1
#define temp_regen 190
#define temp_inject 220
#define FW_version "PrimoSamp-v1.1.0"
#define Trtd_max 250
#define button_pushed_long 50
#define PWM2COM_High_Trigger 4999
#include<aduc7061.h>
# include "stdio.h"
# include "string.h"
# include "stdlib.h"
# include "stdbool.h"

volatile long ulADC0Result = 0;		// Variable which holds ADC0DAT value
volatile long ulADC1Result = 0;		// Variable which holds ADC1DAT value
volatile unsigned char bSendResultToUART = 0;	// Flag used to indicate ADC0 resutl ready to send to UART				// Used to store ADC0 result before printing to UART
volatile unsigned char newADCdata;          // 1 means new ADC data available
char RX_string[10] ="";
char Trtd_string[5] = ""; 
unsigned char szTxData[] = {0x00, 0x38, 0x39, 0x14, 0x70, 0x5D, 0x6D, 0x0F, 0x01};// Array to send to Slave  command + RS + RS + BIAS + Contrast low byte + ICON/Contrast high byte + Follower circuit + Display on	+ Clear Display
unsigned char ucTxCount = 0;  // Array index variable for szTxData[]
unsigned char szRxData[6];	  // Array for reading Data from Slave
unsigned char RX_I2C = 0;
unsigned char ucRxCount = 0;  // Array index variable for szRxData[]
unsigned char ucWaitingForXIRQ0 = 1; // Flag to begin reading from Slave - controlled by XIRQ0
unsigned char check = 0;
unsigned char RX_char = 0;
unsigned int RX_buffer = 0;
bool LCD_connected = true;
float  Trtd = 25.0;		// temperature of RTD
float Rrtd = 0.0;		// Resistance of RTD
float V_out = 0.0;		// Voltage read on ADC2/3
const float T0 = 0.0;
const float R0 = 100.0; //const float R0 = 108.8;
const float alpha = 0.0039; //const float alpha = 0.0039;
unsigned int READ_heat_temp = 0;
unsigned int READ_heat_time = 0;
unsigned char READ_Gain = 0;
unsigned int old_value = 0;
unsigned char PWR_OFF = 1;

const unsigned int const_heat_temp = 80;
const unsigned int KP = 500;	  //, KP, KI and KD are constant of PID correctorKP = 500, KI = 400: small oscillation
const unsigned int KI = 250;
const unsigned int KD = 0;
float error_Trtd = 0.0;
float previous_error_Trtd = 0.0;
float integral_error_Trtd = 0.0;
float derivative_error_Trtd = 0.0;
float output = 0.0;

bool I2C_Rx_received = false;
bool I2C_Tx_sent = false;
bool Timer0_expired = false;
bool Timer0_started = false;
bool loop_count_actived = true;

//***functions declaration***//
void pump_off(void);  
void pump_on(void);
void heater_full_off(void);  
void heater_full_on(void);
void Temperature (void);
int Read_UART_Parameter( unsigned char end_char );
void delay(int);
void I2C_LCD_set_cursor(unsigned char row, unsigned char col);
void I2C_LCD_write_char(char c);
void I2C_LCD_write_string(char str[16]);
void I2C_LCD_write_integer(int number);
void Trtd_string_convert(void);
//***functions declaration***//
 
int main(void)
{
    unsigned int loop_LCD_check = 0;
	unsigned char i;
	unsigned char nLen = 0;
	unsigned char szTemp[64] = "";
	unsigned char state = 0;
	unsigned char B1_counter = 0; // Green Red button
	unsigned char B2_counter = 0; // Green Blue button
	unsigned int elapsed_sec = 0;
	unsigned char elapsed_min = 0;
	unsigned char blink_count = 9;
	unsigned char I2CMTX_temp = BIT0 + BIT4 + BIT7;
	bool B1_pushed = false;
	bool B2_pushed = false;
	bool B1_3s_pushed = false;
	bool B2_3s_pushed = false;
	bool B1_1s_pushed = false;
	bool B2_1s_pushed = false;
	bool T_reach = false;
	bool PID_start = false;
	bool cadtridge_present = true;
	const float T_sampling = 0.05; 
	//bool I_enable = false;
	//bool D_enable = false;
	const unsigned int T_delay = 6500;
	unsigned int loop_count = 0;
	READ_heat_temp = const_heat_temp;
	POWKEY1 = 0x1;
	POWCON0 = 0x78;		   // Set core to max CPU speed of 10.24Mhz
	POWKEY2 = 0xF4;
	//IEXCON = BIT1 + BIT6; 	// Enable Excitation Current Source 200uA for the RTD PT100
	IEXCON = BIT1 + BIT6 + BIT7; 	// Enable Excitation Current Sources 200uA for the RTD PT100
	GP1CON = BIT0 + BIT4 + BIT24;  // Configurer P1.0/P1.1 tant que entrée/sortie UART, pour établir la communication en port série
	
	GP2CON  = BIT0 + BIT4; 	// Configurer P2.0 tant que sortie PWM0 (sortie Modulation de Largeur d'Impulsion pour hacher le transistor qui va alimenter l'élément chauffant du préconcentrateur)
	PWMCON	= BIT0 + BIT6 + BIT7 + BIT8; //
	//the photo MOSFET which is used to switch the pump is connected to output PWM0, to turn pump off, set PWM0COM0 and PWM0COM1 and PWM0LEN as below
	PWM0COM0 = 0x9C40;		//	Configure PWM0 output high trigger time
	PWM0COM1 = 0x0000;		//	Configure PWM0 output Low trigger time,
	//to turn pump on, swap the values of PWM0COM0 and PWM0COM1
	PWM0LEN = 0x9C40;   //1 period/sec , PWM_CLK = 40 kHz

	PWM2COM0 = PWM2COM_High_Trigger;		//	Configure PWM0 output high trigger time
	PWM2COM1 = 0;		//	Configure PWM0 output Low trigger time
	PWM2LEN = PWM2COM_High_Trigger;   //PWM_CLK = 40 kHz, 40000/2000 = 20 PWM period/sec
			
	COMCON0 = BIT7;			// Enable access to COMDIV registers
	COMDIV0 = 0x8;			// Set baud rate to 38400.
	COMDIV1 = 0x00;
	//COMDIV2 = 0x21 + BIT11; // Enable fractional divider for more accurate baud rate setting
	COMCON0 = BIT0 + BIT1 + BIT2;
	ADCMSKI = BIT0 + BIT1;// Enable ADC0 result ready interrupt source
	//ADCFLT = BIT2 + BIT10 + BIT14 + BIT15; //232 Hz, AF = 4, SF = 4
	ADCFLT = BIT1 + BIT3 + BIT8 + BIT11 + BIT12 + BIT14 + BIT15;
	ADC0CON = BIT15;   // Gain = 32, Unipolar, enable ADC0, Int Reference
	ADC1CON = BIT11 + BIT15;
	ADCCFG = 0;	
	// Offest Self Calibration
	ADCMDE  = BIT2 + BIT4 + BIT7; // ADCCLK = 512 kHz, normal mode,	offset calibration
  	while(((ADCSTA & BIT0) != BIT0) && ((ADCSTA & BIT1) != BIT1)){}// 	// Wait for Calibration routine to complete
    
	// Gain Self Calibration
	ADCMDE  = BIT0 + BIT2 + BIT4 + BIT7;	
  	while(((ADCSTA & BIT0) != BIT0) && ((ADCSTA & BIT1) != BIT1)){}//	// Wait for Calibration routine to complete
	
	// 2 ADCs are available, but only 1 is used to read the temperature of the PT100. 
	ADC1CON = BIT1 + BIT8 + BIT11 + BIT15;	   // ADC6/7, PGA = 4, VREF int
	//ADC0CON = BIT1 + BIT6 + BIT8 + BIT11 + BIT15; //03 July ADC2/3 diff mode PGA = 4
	ADC0CON = BIT1 + BIT11 + BIT15; //03 July ADC0/1 diff mode PGA = 4
	ADCMDE  = BIT0 + BIT7;					 	// Enable Continuous conversion mode
	

	// Configure P0.1 and P0.3 for I2C mode
	// because the ADuC7060 has a limited number of I/O, I2C bus is used to extend the number of I/O and peripherlas. 
	GP0CON0 = BIT4 + BIT12; // Select SPI/I2C alternative function for P0.1 & P0.3
	GP0KEY1 = 0x7;		   // Write to GP0KEY1
	GP0CON1	= BIT1;		   // Select I2C functionality for P0.1 & P0.3
	GP0KEY2 = 0x13;		   // Write to GP0KEY2
	GP0DAT |= BIT26;
	//GP0DAT &= ~BIT24 ;
	// Enable I2C Master mode, baud rate and interrupt sources
	I2CMCON = BIT0 + BIT4//   // Enable I2C Master + Enable Rx interrupt
			+ BIT5 + BIT8; // Enable Tx interrupt + Enable transmission complete interrupt
	//I2CDIV  = 0x0A0F;	   // Select 400kHz clock rate
	I2CDIV  = 0x3333;	   // Select 100kHz clock rate
	//IRQCONE = BIT1; // int on raising edge
	IRQCONE = BIT0 + BIT1; // int on falling edge
	//IRQEN = BIT15; // + BIT13; // Enable I2C Master and XIRQ0 interrupts
	T0LD  = 5 << 8;	// value-bit shifted left 8-written to T0LD is second, <=> 5 seconds		
	T0CON = BIT3 + BIT5 + BIT6;	 // Configurate Timer 0 in countdown mode, hour:minute:second format,  
	IRQEN = BIT3 + BIT13 + BIT15 + BIT10; // Enable Timer0-BIT3 + IRQ0 interrupt-BIT13 + I2C interrupt-BIT15 + ADC0 interrupt-BIT10
	//IRQEN = BIT13;	   // IRQ0 interrupt
	// Begin Master Transmit sequence
	ucTxCount = 0;
	I2CFSTA = BIT9;			// Flush Master Tx FIFO
	I2CFSTA &= ~BIT9;
	//I2CMTX = 0x00;//I2CMTX = szTxData[ucTxCount++];
	//I2CMTX = BIT1 + BIT3 + BIT5 + BIT7;
	//I2CADR0 = 0x40;   // slave adress
	//----LCD initialization----// 
	delay(50000);
	I2CADR0 = 0x7C;
	I2CMTX = szTxData[ucTxCount++];
	while((I2C_Tx_sent == 0) && (loop_LCD_check < 100000))
	{
		loop_LCD_check++;
		if (loop_LCD_check == 99999)
		{
		LCD_connected = false;
		}
	}
	loop_LCD_check = 0;
	I2C_Tx_sent = 0;
	I2CMTX = 0x00;	 // command
	delay(1);
	I2CMTX = BIT0; // clear display
	I2CMTX = BIT2 + BIT4; // entry mode set-increment
	//----LCD initialization----// 

	//I2C_LCD_set_cursor(1, 3);
	//I2C_LCD_write_string(FW_version);
	//I2C_LCD_set_cursor(2, 3);	  
	
	ucWaitingForXIRQ0 = 1;
	ucRxCount = 0;
	//I2CMCNT0 = 0;
	//I2CADR0 =  0x41;
	//RX_I2C = I2CMRX;
	I2CADR0 = 0x40;
	I2C_Tx_sent = 0;
	I2CMTX = BIT0 + BIT1 + BIT2 + BIT4;	// set color of LED 2 from Red to Green,adding pump BIT0
	while ( I2C_Tx_sent == 0 ){}
	
	//for (i=0; i<100; i++)  // tempo between 2 R/W operations on I2C bus 
	//{}
	while (1)
	{
	 	
		if (loop_count_actived == true)	// si on met la code directement dans la boucle while, elle se répète à chaque itération de la boucle, par conséquent on introduit une variable loop_count qui s'incrémente à quaque iération, la code principale ne s'exécute que si loop_count atteint une certaine valeur. 		
		{
			loop_count++;
		}
		if (((COMSTA0 & 0x01) == 0x01) || (loop_count == T_delay))	  // le 1er bit de COMSTA0 se met à 1 quand le µcontrôleur reçoit 1 caractère envoyé par l'interface LabVIEW (soit par lien USB, soit par Bluetooth)
	    {	
	
			RX_buffer = COMRX;	// lecture du caractère reçu par le port série
			RX_char = RX_buffer;
			if (RX_char == '*') // si on quitte l'application LabVIEW, un caractère '*' est envoyé à ADuC7060, timing loop sera désormais défini par ADuC7060  
			{
				loop_count_actived = true;
			}
			if (RX_char == '{') // vérifier si le caractère lu est un '{'
			{
			/******Disable Heating Time Settings from labVIEW interface******/
			//	READ_heat_temp = Read_UART_Parameter('}'); //Extraire la valeur de consigne de temp pour le préconcentrateur
			}   		
			
			if (RX_char == '[') // Wait for the beginning character
			{
			/******Disable Heating Time Settings from labVIEW interface******/
			/*
			READ_heat_time = Read_UART_Parameter(']');

				if (old_value != READ_heat_time)
				{
					PWR_OFF = 0;
					T0LD  = READ_heat_time << 8;			
					T0CON = BIT3 + BIT5 + BIT6 + BIT7;	// Periodic mode, enable timer, 32768hz clock/1
					old_value = READ_heat_time;
				}
			*/
			/******Disable Heating Time Settings from labVIEW interface******/
			}

			if (RX_char == '(') // Wait for the beginning character
			{			
				READ_Gain = Read_UART_Parameter(')');
				//ADC0CON = BIT6 + BIT8 + BIT11 + BIT15 + READ_Gain;
				ADC0CON = BIT11 + BIT15 + READ_Gain;
			}			
			
			switch (RX_buffer)
			{	  				   
				case 72: // ASCII "H" character, received from labVIEW, for choosing 200Hz 
				ADCFLT = BIT2 + BIT10 + BIT14 + BIT15;	// SF = 4,AF = 4
				break;

				case 76: // ASCII "L" character, received from labVIEW, for choosing 20Hz sampling
				ADCFLT = BIT1 + BIT3 + BIT8 + BIT11 + BIT12 + BIT14 + BIT15; //26 Hz, SF = 10, AF = 25
				break;

				case 77: // ASCII "M" character, received from labVIEW, for choosing 100Hz sampling
				ADCFLT = BIT2 + BIT9 + BIT11 + BIT14 + BIT15; //123 Hz, SF = 4, AF = 10
				break;

				default:
				break; 
			}
					
				if ((RX_buffer == '{')||(RX_buffer == 'A')||(RX_buffer == 'B')||(RX_buffer == 'C')||loop_count == T_delay)	// , 
				{
					if (RX_buffer == '{')
					{
						loop_count_actived = false;
					}
				//loop_count++;
				if (loop_count == T_delay)  //almost a delay of 50 ms
				{
					loop_count = 0;
				}
				I2CMCNT0 = 0;
				I2CADR0 =  0x49;   //faire 1 lecture des valeurs des 8 entrées DIG du PCF8574 située à l'addresse 0X49 
				while (I2C_Rx_received == false) //attendre l'interruption pour que les données soient bien reçues
				{
				  //I2C_Rx_received = true;
				}
				
				if ((RX_I2C & BIT0) == 0)
				{
					B1_pushed = true;
					B1_counter++;
				}
				else
				{
					if ( (B1_counter < 20) && (B1_pushed == true) )
					{
						B1_1s_pushed = true;
						B1_counter = 0;
					}
					if ( (B1_counter >= 20) && (B1_counter <= button_pushed_long) )
					{
						B1_pushed = false;
						B1_counter = 0;
					}
				}
				if ((RX_I2C & BIT1) == 0)
				{
					B2_pushed = true;
					B2_counter++;
				}
				else
				{
					if ( (B2_counter < 20) && (B2_pushed == true) )
					{
						B2_1s_pushed = true;
						B2_counter = 0;
					}
					if ( (B2_counter >= 20) && (B2_counter <= button_pushed_long) )
					{
						B2_pushed = false;
						B2_counter = 0;
					}									
				}
				if (( B1_counter > button_pushed_long)&&(state == 0))
				{
					B1_3s_pushed = true;
					B1_counter = 0;
				}
				
				if (( B2_counter > button_pushed_long)&&(state == 0))
				{
					B2_3s_pushed = true;
					B2_counter = 0;
				}										
													
				switch (state)
				{
					case 0:	// initial state | LED1 GREEN + LED2 GREEN
 					
					
					if ((Trtd > 0)&&(Trtd < 1000))
					{					  
						cadtridge_present = true;
						I2CADR0 = 0x40;
						I2C_Tx_sent = false;
						I2CMTX = BIT0 + BIT4 + BIT7;
						while ( I2C_Tx_sent == false ){}
					}
					else
					{  
						cadtridge_present = false;
						blink_count++;
						B1_counter = 0;		//v1.1.4
						B2_counter = 0; 	//v1.1.4
						if (blink_count == 10)
						{
							blink_count = 0;
							I2CMTX_temp ^= (BIT3 + BIT4);
							I2CADR0 = 0x40;
							I2C_Tx_sent = false;
							I2CMTX = I2CMTX_temp;
							while ( I2C_Tx_sent == false ){}
						}

					}
					
						//I2CADR0 = 0x40;
						//I2CMTX = BIT0 + BIT4 + BIT7;
						//while ( I2C_Tx_sent == 0 ){}
						//I2C_Tx_sent = 0;
					pump_off();
					heater_full_off();
					T_reach = false;
					PID_start = false;
					error_Trtd = 0;
					integral_error_Trtd	= 0;
					derivative_error_Trtd = 0;
					I2C_LCD_set_cursor(1, 1);
					I2C_LCD_set_cursor(1, 1);
					//I2C_LCD_write_string("STAND BY    ");
					I2C_LCD_write_string("v1.1.4  STAND BY");
					Timer0_started = false;
					Timer0_expired = false; 
					if ((B1_pushed == true) && (B1_3s_pushed == true) && (cadtridge_present == true))	 // presssing more than 3S on B1 => RESET
					{
						state = 1;
						B1_pushed = false;
						B1_3s_pushed = false;
					}
					if ((B2_pushed == true) && (B2_3s_pushed == true) && (cadtridge_present == true))	// presing more than 3s on B2 => INJECT
					{
						state = 3;
						B2_pushed = false;
						B2_3s_pushed = false;
					}
					if ((B2_pushed == true) && (B2_1s_pushed == true))	// pressing less than 1s on B2 => SAMPLE
					{
						state = 2;
						B2_pushed = false;
						B2_1s_pushed = false;
					}
					break;
				
					case 1: // RESET | LED1 RED + LED2 GREEN
 					I2CADR0 = 0x40;
					I2CMTX = BIT1 + BIT2 + BIT3 + BIT4; // PWM4 output is on PWM4 MOSFET switch by default, add '+ BIT7' if select PWM4 on PWM extra MOSFET 
					while ( I2C_Tx_sent == 0 ){}
					I2C_Tx_sent = 0;
					pump_on();
					//heater_full_on();
					I2C_LCD_set_cursor(1, 1);
					I2C_LCD_write_string("v1.1.4     REGEN");//I2C_LCD_write_string("RESET   ");
					if (Timer0_started == false)
					{
					Timer0_started = true;
					T0LD  = (1 << 16) + (30 << 8);	// Loading Timer duration, T = 1 min + 30s
					//T0LD  = 10 << 8; // Loading Timer duration, T = 10 s		
					T0CON = BIT3 + BIT5 + BIT6 + BIT7;
					}
					
					//---PID Code---//
					/*
					if (Trtd  < temp_regen)
					{
						heater_full_on();
					}
					else
					{
						heater_full_off();
					}
					*/
					//------Asservissement de la température du préconcentrateur en mode REGENERATION//
					if ((Trtd  < temp_regen-10) && (T_reach == false)) // si la température lue du PT100 n'a pas encore atteint la consigne et reste < à la consigne - 10°C, la commande est calculée par un correcteur P (proportionel), on évite d'utiliser le terme I (intégral) dès au début pour éviter que la saturation de la commande peut entrainer l'accumulation des erreurs, ce qui cause de grands dépassement/oscillations (integrator windings)
					{
						error_Trtd = temp_regen - Trtd; 
						output = KP*error_Trtd;
						previous_error_Trtd = error_Trtd;
						if ((output >= 0)&&(output <= PWM2COM_High_Trigger))
						{
							PWM2COM1 = (int) output;
						}
						else if (output > PWM2COM_High_Trigger)
						{
							PWM2COM1 = PWM2COM_High_Trigger;	//PW2COM1 ne peut pas dépasser sa valeur max, ce qui est PW2COM_High_Trigger
						}
						//I_enable = false;
						//D_enable = false//set flag to enable only P corrector, disable I and D corrector
					}
					else if	 ((Trtd  >= (temp_regen)) && (T_reach == false)) // si la température lue du PT100 dépasse la consigne pour la 1ère fois, arrêter à chauffer et mettre à 1 la valeur du flag T_reach
					{
						T_reach = true;
						heater_full_off();
					}
					if ((Trtd  < temp_regen) && (T_reach == true)) // si la température lue PT100 chute et < la consigne, alors la commande sera calculée par 1 PID (proportionel + intégral + différentiel)
					{
						PID_start = true;
					}
					if (PID_start == true)
					{
						//I_enable = true;
						//D_enable = true;
						error_Trtd = temp_regen - Trtd;	 // calcul des termes P, I, D
						integral_error_Trtd	= integral_error_Trtd + error_Trtd*T_sampling;
						derivative_error_Trtd = (error_Trtd - previous_error_Trtd)/T_sampling;
						output = KP*error_Trtd + KI*integral_error_Trtd + KD*derivative_error_Trtd;	 // et calcul de la somme des termes P, I et D
						output = KP*error_Trtd;
						previous_error_Trtd = error_Trtd;
						if ((output >= 0)&&(output <= PWM2COM_High_Trigger))
						{
							PWM2COM1 = output;
						}
						if (output < 0)
						{
							PWM2COM1 = 0;
						}
						if (output > PWM2COM_High_Trigger)
						{
							PWM2COM1 = PWM2COM_High_Trigger; //la commande ne peut pas dépasser la valeur min 0 et la valeur max PW2COM1 
						}
						//set flag to enable all PID corrector						
					}					
					//I2C_LCD_set_cursor(1, 12);
					//I2C_LCD_write_integer((int)integral_error_Trtd);
					//---PID Code---//

					if ((((B1_pushed == true) && (B1_1s_pushed == true) && (B2_pushed == true) && (B2_1s_pushed == true)) || (Timer0_expired == true) || (Trtd > Trtd_max)) && (state != 4))
					{
						state = 0;
						B1_pushed = false;
						B1_1s_pushed = false;
						B1_3s_pushed = false;
						B1_counter = 0;		//v1.1.4
						B2_pushed = false;
						B2_1s_pushed = false;
						B2_3s_pushed = false;
						B2_counter = 0;		//v1.1.4									 
					}
					;
					
					break;
						   				
					case 2: // SAMPLE | LED1 GREEN + LED2 BLUE
				 	I2CADR0 = 0x40;
					I2CMTX = BIT1 + BIT2 + BIT7;
					while ( I2C_Tx_sent == 0 ){}
					I2C_Tx_sent = 0;					
					pump_on();
					heater_full_off();
					I2C_LCD_set_cursor(1, 1);
					I2C_LCD_write_string("v1.1.4  SAMPLING");
					//if (Timer0_started == false)
					//{
					//Timer0_started = true;
					//T0LD  = 7 << 8;			
					//T0CON = BIT3 + BIT5 + BIT6 + BIT7;
					//}
					//if (((B1_pushed == true) && (B1_1s_pushed == true) && (B2_pushed == true) && (B2_1s_pushed == true)) || (Timer0_expired == true) )
					if ((B2_pushed == true) && (B2_1s_pushed == true))
					{
						state = 0;
						B1_pushed = false;
						B1_1s_pushed = false;
						B2_pushed = false;
						B2_1s_pushed = false;
					}
					break;

					case 3: // INJECT | LED1 RED + LED2 BLUE
 					I2CADR0 = 0x40;
					I2CMTX = BIT0 + BIT3;
					while ( I2C_Tx_sent == 0 ){}
					I2C_Tx_sent = 0;					
					pump_off();
					I2C_LCD_set_cursor(1, 1);
					I2C_LCD_write_string("v1.1.4 INJECTION");


					//heater_full_on();
					if (Timer0_started == false)
					{
					Timer0_started = true;
					//T0LD  = (1 << 16) + (30 << 8);	// 1 min + 30s
					T0LD  = (15 << 8);	// 15s		
					T0CON = BIT3 + BIT5 + BIT6 + BIT7;
					}
					
					//---simple switch on/off code---//
					/*
					if (Trtd  < (READ_heat_temp))
					{
						heater_full_on();						
					}					

					if (Trtd  > (READ_heat_temp))
					{
						heater_full_off();						
					}
					*/					
					//---simple switch on/off code---//					
					//---PID code---//
					
					
					if ((Trtd  < (temp_inject-10)) && (T_reach == false))
					{
						heater_full_on();	 					
					}
					else if ((Trtd  >= (temp_inject-10)) && (Trtd  < temp_inject) &&(T_reach == false))
					{
						error_Trtd = temp_inject - Trtd;
						output = KP*error_Trtd;
						previous_error_Trtd = error_Trtd;
						if ((output >= 0)&&(output <= PWM2COM_High_Trigger))
						{
							PWM2COM1 = output;
						}
						if (output < 0)
						{
							PWM2COM1 = 0;
						}
						if (output > PWM2COM_High_Trigger)
						{
							PWM2COM1 = PWM2COM_High_Trigger;
						}
						//I_enable = false;
						//D_enable = false//set flag to enable only P corrector, disable I and D corrector
					}
					else if	 ((Trtd  >= (temp_inject)) && (T_reach == false))
					{
						T_reach = true;
						heater_full_off();
					}
					if ((Trtd  < temp_inject) && (T_reach == true))
					{
						PID_start = true;
					}
					if (PID_start == true)
					{
						//I_enable = true;
						//D_enable = true;
						error_Trtd = temp_inject - Trtd;
						integral_error_Trtd	= integral_error_Trtd + error_Trtd*T_sampling;
						derivative_error_Trtd = (error_Trtd - previous_error_Trtd)/T_sampling;
						output = KP*error_Trtd + KI*integral_error_Trtd + KD*derivative_error_Trtd;
						output = KP*error_Trtd;
						previous_error_Trtd = error_Trtd;
						if ((output >= 0)&&(output <= PWM2COM_High_Trigger))
						{
							PWM2COM1 = output;
						}
						//set flag to enable all PID corrector						
					}					

					//***PID code***//					
					if ((((B1_pushed == true) && (B1_1s_pushed == true) && (B2_pushed == true) && (B2_1s_pushed == true)) || (Timer0_expired == true) || (Trtd > Trtd_max)) && (state != 4))
					
					{
						state = 0;
						B1_pushed = false;
						B1_1s_pushed = false;
						B1_3s_pushed = false;
						B1_counter = 0;		//v1.1.4
						B2_pushed = false;
						B2_1s_pushed = false;
						B2_3s_pushed = false;
						B2_counter = 0;		//v1.1.4
						T0CON &=!BIT7;
						T0LD  = 0 << 8;
					}
					break;
					case 4:	// error state
				 	//blink_count++;
					/*I2CADR0 = 0x40;
					I2CMTX ^= (BIT3+BIT4);
					while ( I2C_Tx_sent == 0 ){}
					I2C_Tx_sent = 0;										
					if (blink_count > 100)
					{
						I2CADR0 = 0x40;
						I2CMTX ^= BIT3;
						while ( I2C_Tx_sent == 0 ){}
						I2C_Tx_sent = 0;
						blink_count = 0;
					}

					pump_off();
					heater_full_off();
					B1_pushed = false;
					B1_1s_pushed = false;
					B2_pushed = false;
					B2_1s_pushed = false;
					I2C_LCD_set_cursor(1, 1);
					I2C_LCD_write_string("ERROR       ");*/											
					break;

					default :
				
					break; 
				}
				I2C_LCD_set_cursor(2, 1);
				I2C_LCD_write_string("RTD:");
				Trtd_string_convert();  
				I2C_LCD_write_string(Trtd_string);
				/*if (state == 4)
				{
					blink_count++;
					if (blink_count > 20)
					{
						I2CADR0 = 0x40;
						I2CMTX ^= BIT3;	 //BIT0 + BIT4 + BIT7;
						while ( I2C_Tx_sent == 0 ){}
						I2C_Tx_sent = 0;
						blink_count = 0;
					}
				}*/
				//elapsed_sec = T0VAL >> 8;
				//elapsed_sec = 60*(((T0VAL & BIT16) >> 16) + ((T0VAL & BIT14) >> 14) + ((T0VAL & BIT18) >> 18)) + (T0VAL & (BIT8 + BIT9 + BIT10 + BIT11 + BIT12 + BIT13)) >> 8;
				//elapsed_sec = (int)((T0VAL & BIT16) >> 10)*60 + (T0VAL & (BIT8 + BIT9 + BIT10 + BIT11 + BIT12 + BIT13)) >> 8;
				elapsed_min = ((T0VAL & BIT16) >> 16);
				elapsed_sec = (T0VAL & (BIT8 + BIT9 + BIT10 + BIT11 + BIT12 + BIT13)) >> 8;
				elapsed_sec = elapsed_sec + elapsed_min*60;// + (T0VAL & (BIT8 + BIT9 + BIT10 + BIT11 + BIT12 + BIT13)) >> 8;
				I2C_LCD_set_cursor(2, 12);
				I2C_LCD_write_integer(elapsed_sec);
				//while ((I2CMSTA & BIT2) == BIT2)	  // Attendre jusqu'à ce que Tx buffer est vide
	      		//{
	      		//}
				//sprintf ( (char*)szTemp, "byte%2lX %2lX %2lX %2lX %2lX %2lX %2lX %2lX %2lX %2lX %2lX \r\n", state, T0VAL, B1_pushed, B2_pushed, B1_counter, B2_counter, B1_1s_pushed, B2_1s_pushed, B1_3s_pushed, B2_3s_pushed, Timer0_expired );// Envoyer l'information sur la tension du pont TCD et la temp du préconcentrateur au port
				if 	(newADCdata == 1)
				{
				newADCdata = 0;
				Temperature();
				sprintf ( (char*)szTemp, "T%6lXV%6lX S%2lXD%2lX, %3.1f °C P%d\r\n", ulADC1Result, ulADC0Result, state, PWM2COM1, Trtd, cadtridge_present );// Envoyer l'information sur la tension du pont TCD et la temp du préconcentrateur au port                                                   
       			//sprintf ( (char*)szTemp, "byte%2lX\r\n", READ_heat_time );
   				nLen = strlen((char*)szTemp);
     				for ( i = 0 ; i < nLen ; i++ )	
   					{
	     		 	COMTX = szTemp[i];
					while ((COMSTA0 & 0x40) == 0x00)	  // Attendre jusqu'à ce que Tx buffer est vide
	      			{
	      			}
					}
				IRQEN |= BIT10;
				}
			}			   	
			
		
		}
			//if (check == 1)
		    //{
		   	//check = 0;
		   	//IRQEN |= BIT13;
			//}
	}
	


   return 0x00;
}



void IRQ_Handler(void) __irq
{
	unsigned long IRQSTATUS = 0;
	unsigned int I2CMSTATUS = 0;
	unsigned char i = 0;
	//int length = 1000000;
	IRQSTATUS = IRQSTA;	   // Read off IRQSTA register
	if ((IRQSTATUS & BIT3) == BIT3)	//Si l'interruption est causée par Timer0 (chaque fois le timer expire)
	{
	 
			 Timer0_expired = true;
			 T0CON &=!BIT7;
			 T0CLRI = 0x55;		// Clear the currently active Timer0 Irq	 
	}
	if ((IRQSTATUS & BIT13) == BIT13)	//XIRQ0 interrupt source
	{
		IRQCLRE = BIT13;
		//IRQEN &= ~BIT13;
		//if (check == 0)
		//{
		check = 1;
		//GP0DAT ^= BIT18;
		for (i=0; i<100; i++)  // tempo between 2 R/W operations on I2C bus 
		{}
		//I2CMCNT0 = 0;
		//I2CADR0 =  0x49;
		for (i=0; i<100; i++)  // tempo between 2 R/W operations on I2C bus 
		{}
		//RX_I2C = I2CMRX;
		for (i=0; i<100; i++)  // tempo between 2 R/W operations on I2C bus 
		{}
		//if 	 ((RX_I2C & BIT0) == 0x00)	// check on which button comes the interrupt source ?  case 1: button 1	R/G
		//{
		//temp_READ_int = READ_heat_time;
		//temp_READ_int ^= BIT3;
	   	//I2CMTX = temp_READ_int;
		//I2CADR0 = 0x40;
		//I2CMTX ^= BIT3;
		//}
		for (i=0; i<100; i++)
		{}
		//if 	 ((RX_I2C & BIT1) == 0x01)	// check on which button comes the interrupt source ? case 2: button 2 B/G
		//{
		//temp_READ_int = READ_heat_time;
		//temp_READ_int ^= BIT3;
	   	//I2CMTX = temp_READ_int;
		
		//I2CADR0 = 0x40;
		//I2CMTX ^= BIT4;
		//}

		ucWaitingForXIRQ0 = 0;			// Enable Read request from Slave.			
		//}
	}

	if ((IRQSTATUS & BIT15) == BIT15)	//If I2C Master interrupt source
	{
	   I2CMSTATUS = I2CMSTA;
	   if ((I2CMSTATUS & BIT2) == BIT2) // If I2C Master Tx IRQ
	   {
			if (ucTxCount < 9)		   // Have max 9 bytes been sent?
				I2CMTX = szTxData[ucTxCount++];	 // Load Tx buffer
			//else			
			//	I2C_Tx_sent = true;
	   }
	   if ((I2CMSTATUS & BIT8) == BIT8) // If I2C Master Transmission completed
	   {			
				I2C_Tx_sent = true;
	   }
	   if ((I2CMSTATUS & BIT3) == BIT3) // If I2C Master Rx IRQ
	   {
			RX_I2C = I2CMRX;
			I2C_Rx_received = true;
			//if (ucRxCount < 6)		   // Have max 6 bytes been received?
			//{
			//	RX_I2C = I2CMRX;  // Read Rx buffer
			//	ucRxCount++;
			//}
	   }
	}
	if ((IRQSTATUS & BIT10) == BIT10)	//Si l'interruption est causée par ADC (chaque fois 1 nouvelle conversion est disponible) 
	{
	  	// if ADCnorm variables are ready for new data..
  		if (newADCdata==0) 
		{
			ulADC1Result = ADC1DAT;	// Lecture du résultat de conversion de ADC1 
			ulADC0Result = ADC0DAT;	// Lecture du résultat de conversion de ADC0
  		}

 		newADCdata = 1;				// mettre à 1 le sémaphore newADCdata
		IRQCLR = BIT10;				// Disable ADC interruption
	}
}
void pump_off()
{
	PWM0COM0 = 0x9C40;		//	Configure PWM0 output high trigger time
	PWM0COM1 = 0x0000;		//	Configure PWM0 output Low trigger time,
}
void pump_on()
{
	PWM0COM0 = 0x0000;		//	Configure PWM0 output high trigger time
	PWM0COM1 = 0x9C40;		//	Configure PWM0 output Low trigger time,
}
void heater_full_off(void)
{
	PWM2COM0 = PWM2COM_High_Trigger;		//	Configure PWM2 output high trigger time
	PWM2COM1 = 0;		//	Configure PWM2 output Low trigger time,
}
void heater_full_on()
{
	PWM2COM0 = PWM2COM_High_Trigger;//PWM2COM0 = 0;		//	Configure PWM0 output high trigger time
	PWM2COM1 = PWM2COM_High_Trigger;//PWM2COM1 = PWM2COM_High_Trigger;		//	Configure PWM0 output Low trigger time,
}  
void Temperature () 
{
  // First calculate the resistance across the RTD
  // Equation = ADC0Result * ((RREF/Gain)/# of bits)
  //Rrtd = (float)ulADC0Result * ((2.5) /0xFFFFFF);
  //Rrtd = (float)ulADC1Result * ((5600.0 /4.0) /0xFFFFFF); //  * (1.2/1.15)
    //Rrtd = (float)ulADC1Result * ((1000.0 /4.0) /0xFFFFFF); // ADC1 voltage in mV
  V_out = (float)ulADC0Result * (1200.0/4.0) /0x7FFFFF;	  // ADC1 voltage in mV, in bipolar mode, the reference for a positive voltage is 0x7FFFFF 
  Rrtd = V_out/0.2; 
  // At 35 degree Celsius, the equivalent resistance of a PT100 is 113.607 Ohm.
  //Trtd = (Rrtd - 184.72)/0.3657;	//ATTENTION, ce calcul est fait pour le RTD PT100 de ADµC7061, au cas du RTD PT100, il faut modifier de calcul car ce dernier possède un coef alpha plus faible 
  Trtd = (Rrtd - R0 + R0*T0*alpha)/(R0*alpha);
}
int Read_UART_Parameter(unsigned char end_char)
{
			unsigned int k;
			unsigned int i;
	
				k = 0;
				for (i=0; i<10; i++)
				{ 
					RX_string[i]='\0';
				}                				
		   		while ((COMSTA0 & 0x01) == 0x00)	  // Wait for COMRX buffer empty bit to be set
	      		{
	      		}				
				RX_char = COMRX;
				while (RX_char != end_char)
				{ 					
					if ((RX_char == '0')||(RX_char == '1')||(RX_char == '2')||(RX_char == '3')||(RX_char == '4')||(RX_char == '5')||(RX_char == '6')||(RX_char == '7')||(RX_char == '8')||(RX_char == '9'))
					RX_string[k] = RX_char;	
					k++;
					while ((COMSTA0 & 0x01) == 0x00)
					{
					}
					RX_char = COMRX;					
				}
				//READ_heat_time = atoi(RX_string);
				//PWM0COM0 = PWM0COM1 + READ_heat_time*2;	
	return atoi(RX_string);
}

void delay(int length)
{
	while (length >0)
   	length--;
}


void I2C_LCD_set_cursor(unsigned char row, unsigned char col) 
{
	if (LCD_connected == true)
	{
	I2CADR0 = 0x7C;
	//I2C_Tx_sent = false;
	I2CMTX = 0x00;	 // command
	//while ( I2C_Tx_sent == false ){}
	delay(1);
	I2C_Tx_sent = false;
	I2CMTX = (row - 1)*64 + (col-1) + BIT7;
	while ( I2C_Tx_sent == false ){}
	//delay(100);
	}
}


void I2C_LCD_write_char(char c)
{
	if (LCD_connected == true)
	{ 	
	I2CADR0 = 0x7C;
	I2CMTX = 0x40; //write command
	I2C_Tx_sent = 0;
	I2CMTX = c;
	while ( I2C_Tx_sent == 0 ){}
	I2C_Tx_sent = 0;
	//delay(150); // delay(100) for 4K7 pull up resistor,  delay(150) for 10K pull up resistor
	}
}

void I2C_LCD_write_string(char str[16])
{
 	unsigned char i_char = 0;
	while ( str[i_char] != '\0' )
	{
		I2C_LCD_write_char(str[i_char++]);
	}
}
void I2C_LCD_write_integer(int number)
{
	char number_string[16];
	unsigned char k=0;
	if (LCD_connected == true)
	{	
	for (k=0; k<16; k++)
	{ 
		number_string[k]='\0';
	}
	if (number > 999)
	{
		sprintf( (char*)number_string, "%d  ", number);
	}	
	else if (number > 99)
	{
		sprintf( (char*)number_string, "0%d  ", number);
	}
	else if (number > 9)
	{
		sprintf( (char*)number_string, "00%d  ", number);
	}
	else
	{
		sprintf( (char*)number_string, "000%d  ", number);
	}
	I2C_LCD_write_string(number_string);
	} 
}

void Trtd_string_convert(void)
{
	if (Trtd < 100)
	{
		sprintf( (char*)Trtd_string, "% 2.1f ", Trtd); 
	}
	else
	{
		sprintf( (char*)Trtd_string, "%3.1f ", Trtd);
	}
}

