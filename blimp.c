#include <c8051_SDCC.h>		// Include C8051 SDCC Header
#include <stdio.h>		// Include Standard Input/Output Header
#include <stdlib.h>		// Include Standard Library Header
#include <i2c.h>		// Include I2C Bus Header

#define PW_MIN 2000
#define PW_NEUT 2750
#define PW_MAX 3500

void Port_Init(void);			// Initialize Ports for Input and Output
void PCA_Init(void);			// Initialize Programmable Counter Array 0
void XBR0_Init(void);			// Initialize Crossbar 0
void SMB_Init(void);			// Initialize SMBus
void ADC_Init(void);			// Initialize Analog/Digital Converter 1
void PCA_ISR(void) interrupt 9;		// Programmable Counter Array 0 Interrupts Subroutine
int ADC_input(void);			// Read Analog/Digital Converter 1 Input
void Steer_Rudder(void);		// Steer Rudder Fan
void Drive_Thrust(void);		// Drive Thrust Fans
int Read_Compass(void);			// Read Compass
unsigned int Read_Ranger(void);		// Read Ranger
void config_blimp(void);		// Configure Blimp
void keypad_wait(void);			// Wait before reading user input from keypad

sbit at 0xB0 CLEAR_SCREEN;	// Clear Screen Slideswitch connected to Port 3 Pin 0 --> input
sbit at 0xB6 DRIVE_SWITCH;	// Drive Slideswitch connected to Port 3 Pin 6 --> input
sbit at 0xB7 STEER_SWITCH;	// Steer Slideswitch connected to Port 3 Pin 7 --> input

unsigned int h_count;			// PCA0 Timer overflow counter for compass
unsigned int r_count;			// PCA0 Timer overflow counter for ranger
unsigned int new_heading;		// Flag for reading car heading data from compass
unsigned int new_range;			// Flag for reading driving speed data from ranger
unsigned int ANGLE_PW = 2700;		// Actual servo motor steering pulsewidth
unsigned int PCA_start = 28672;		// PCA period used for approximately 20ms
unsigned int desired_heading = 900;	// Desired heading value
unsigned int PD_case = 2;		// Case number for steering control
xdata long RUDDER_PW;			// Actual rudder fan dc motor steering pulsewidth
xdata long THRUST_PW;			// Actual thrust fans dc motor driving pulsewidth
xdata int kd_thrust = 50;		// Derivative Gain for Thrust Fans
xdata float kp_thrust = 0.5;		// Proportional Gain for Thrust Fans
int heading;				// Car heading in tenths of a degree between 0 and 3599
int range;				// Ultrasonic Ranger reading
int error;				// Steering needed to reach desired heading
int range_error;			// Thrust Error
int prev_error = 0;			// Previous Error
int prev_range_error = 0;		// Previous Range Error
int battery = 100;			// Percent battery charge left
int PCA_count = 0;			// PCA0 Timer overflow counter
int wait = 0;				// Wait counter
char keypad;				// User input from keypad

// Autonomous Blimp
void main(void) {
	
	Sys_Init();	// Initialize System
	Port_Init();	// Initialize Ports for Input and Output
	PCA_Init();	// Initialize Programmable Counter Array 0
	XBR0_Init();	// Initialize Crossbar 0
	SMB_Init();	// Initialize SMBus
	ADC_Init();	// Initialize Analog/Digital Converter 1
	
	putchar(' ');	// The quote fonts may not copy correctly into SiLabs IDE
	
	RUDDER_PW = PW_NEUT;	// Set Rudder Fan DC Motor to center
	THRUST_PW = PW_NEUT;	// Set Thrust Fans DC Motor to neutral
	
	PCA0CPL0 = 0xFFFF - RUDDER_PW;		// Set low byte of CCM0 --> rudder fan heading
	PCA0CPH0 = (0xFFFF - RUDDER_PW) >> 8;	// Set high byte of CCM0 --> rudder fan heading
	PCA0CPL1 = 0xFFFF - ANGLE_PW;		// Set low byte of CCM1 --> thrust fans angle
	PCA0CPH1 = (0xFFFF - ANGLE_PW) >> 8;	// Set high byte of CCM1 --> thrust fans angle
	PCA0CPL2 = 0xFFFF - THRUST_PW;		// Set low byte of CCM2 --> left thrust fan power
	PCA0CPH2 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM2 --> left thrust fan power
	PCA0CPL3 = 0xFFFF - THRUST_PW;		// Set low byte of CCM3 --> right thrust fan power
	PCA0CPH3 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM3 --> right thrust fan power
	
	// Wait one second for DC Motor to warm up
	while (PCA_count < 50)
		printf("");
	
	delay_time(100000);	// Wait for the keypad and display to initialize
	config_blimp();		// Configure Blimp
	
	printf("\n\rheading");
	printf("\t\tdesired heading");
	printf("\tcompass error");
	printf("\taltitude");
	printf("\tdesired altitude");
	printf("\trange error");
	printf("\tbattery voltage\n\r");
	
	keypad_wait();	// Wait for data to display
	
	while (1) {
		
		Steer_Rudder();		// Steer Rudder Fan
		Drive_Thrust();		// Drive Thrust Fans
		
		battery = ADC_input() * 40 / 74;	// Calculate percent battery charge left
		
		// Check for user input of 0
		if (read_keypad() == '0')
			config_blimp();		// Configure Blimp
		
		// Update display every 1000 ms
		if (PCA_count % 50 == 0) {
			
			lcd_clear();	// Clear display
			lcd_print("Battery: %d percent\r", battery);
			lcd_print("Heading: %u deg\r", heading / 10);
			lcd_print("Range: %d counts\r", range);
			
			if (battery > 10)
				lcd_print("PRESS 0 TO CONFIGURE");
			else
				lcd_print("WARNING: BATTERY LOW");
			
			PCA_count = 0;	// Clear PCA0 Timer overflow counter
			
			printf("%u\t\t", heading / 10);
			printf("%u\t\t", desired_heading / 10);
			printf("%d\t\t", error);
			printf("%u\t\t", range);
			printf("50\t\t");
			printf("%d\t\t", range_error);
			printf("%u\n\r", battery * 74 / 1000);
			
		}
		
		// Read compass every 2 PCA0 Timer overflows (40 ms)
		if (h_count >= 2) {
			new_heading = 1;	// Set compass read flag to 1
			h_count = 0;
		}
		
		// Read ranger every 4 PCA0 Timer overflows (80 ms)
		if (r_count >= 4) {
			new_range = 1;	// Set ranger read flag to 1
			r_count = 0;
		}
		
		// Stop running software if battery almost dead
		if (battery <= 5) {
			
			lcd_clear();	// Clear display
			lcd_print("\r BATTERY LOW!\r");
			lcd_print(" PLEASE RECHARGE!");
			
			RUDDER_PW = PW_NEUT;	// Center rudder fan if battery low
			THRUST_PW = PW_MIN;	// Set thrust fans down if battery low
			
			PCA0CPL0 = 0xFFFF - RUDDER_PW;		// Set low byte of CCM0 --> rudder fan heading
			PCA0CPH0 = (0xFFFF - RUDDER_PW) >> 8;	// Set high byte of CCM0 --> rudder fan heading
			PCA0CPL2 = 0xFFFF - THRUST_PW;		// Set low byte of CCM2 --> left thrust fan power
			PCA0CPH2 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM2 --> left thrust fan power
			PCA0CPL3 = 0xFFFF - THRUST_PW;		// Set low byte of CCM3 --> right thrust fan power
			PCA0CPH3 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM3 --> right thrust fan power
			
			return;
			
		}
		
		// Check if clear screen switch enabled
		if (CLEAR_SCREEN)
			lcd_clear();	// Clear display
		
		wait = 0;	// Clear wait counter
		
		// Loop until wait counter reaches 10
		while (wait < 10) {
			
			wait++;	// Increment wait counter
			printf("");
		
		}
		
	}
	
}

// Initialize Ports for Input and Output
void Port_Init(void) {
	
	P0MDOUT |= 0xF0;	// Set Port 0 Pins 4, 5, 6 and 7 to Push-Pull output mode
	P0MDOUT &= ~0x0C;	// Set Port 0 Pins 2 and 3 to Open-Drain input mode
	P0 |= 0x0C;		// Set Port 0 Pins 2 and 3 to High Impedance mode
	P1MDIN &= ~0x08;	// Set Port 1 Pin 3 to Analog input mode
	P1MDOUT &= ~0x08;	// Set Port 1 Pin 3 to Open-Drain input mode
	P3MDOUT &= ~0xD1;	// Set Port 3 Pins 0, 6 and 7 to Open-Drain input mode
	P3 |= 0xD1;		// Set Port 3 Pins 0, 6 and 7 to High Impedance mode
	
}

// Initialize Programmable Counter Array 0
void PCA_Init(void) {
	
	PCA0MD = 0x81;		// Use SYSCLK/12, enable CF interrupts, suspend when idle
	PCA0CPM0 = 0xC2;	// Set CCM0 to 16-bit compare mode
	PCA0CPM1 = 0xC2;	// Set CCM1 to 16-bit compare mode
	PCA0CPM2 = 0xC2;	// Set CCM2 to 16-bit compare mode
	PCA0CPM3 = 0xC2;	// Set CCM3 to 16-bit compare mode
	PCA0CN |= 0x40;		// Enable PCA0 counter
	EIE1 |= 0x08;		// Enable PCA0 interrupts
	EA = 1;			// Enable global interrupts
	
}

// Initialize Crossbar 0
void XBR0_Init(void) {
	
	XBR0 = 0x25;	// XBR0 enables TX0, RX0, SDA, SCL, CEX0, CEX1, CEX2 and CEX3
	
}

// Initialize SMBus
void SMB_Init(void) {
	
	SMB0CR = 0x93;	// Set SCL to 100kHz
	ENSMB = 1;	// Enable the SMBus
	
}

// Initialize Analog/Digital Converter 1
void ADC_Init(void) {
	
	REF0CN = 0x03;		// Set Vref to use internal reference voltage (2.4 V)
	ADC1CN = 0x80;		// Enable Analog/Digital Converter (ADC1)
	ADC1CF |= 0x01;		// Set Analog/Digital Converter gain to 1
	
}

// Programmable Counter Array 0 Interrupts Subroutine
void PCA_ISR(void) interrupt 9 {
	
	if (CF) {
		
		PCA_count++;	// Increment PCA0 Timer overflow counter
		h_count++;	// Increment PCA0 Timer overflow counter for compass
		r_count++;	// Increment PCA0 Timer overflow counter for ranger
		
		PCA0L = PCA_start;		// Set low byte of PCA0 start count
		PCA0H = PCA_start >> 8;		// Set high byte of PCA0 start count
		CF = 0;				// Clear interrupt flag
		
	} else
		PCA0CN &= 0xC0;		// Handle all other type 9 interrupts
	
}

// Read Analog/Digital Converter 1 Input
int ADC_input(void) {
	
	AMX1SL = 3;			// Set Port 1 Pin 3 as Analog Input for ADC1
	ADC1CN = ADC1CN & ~0x20;	// Clear the Conversion Completed flag
	ADC1CN = ADC1CN | 0x10;		// Initiate Analog/Digital conversion
	
	while ((ADC1CN & 0x20) == 0x00);	// Wait for conversion to complete
	
	return ADC1;	// Return digital value in ADC1 register
	
}

// Steer Rudder Fan
void Steer_Rudder(void) {
	
	if (new_heading) {
		
		heading = Read_Compass();	// Read Compass
		
		// Determine steering needed to reach desired heading
		error = desired_heading - heading;
		
		// Correct error for degrees less than -180
		if (error < -1800)
			error = 3600 + error;
		
		// Correct error for degrees greater than 180
		if (error > 1800)
			error = error - 3600;
		
		// Get Servo Motor pulsewidth
		if (PD_case == 1)
			RUDDER_PW = (long) PW_NEUT + (long) (0.5) * (long) (error) + (long) (70) *(long) (error - prev_error);	// Case 1
		
		else if (PD_case == 2)
			RUDDER_PW = (long) PW_NEUT + (long) (2) * (long) (error) + (long) (100) *(long) (error - prev_error);	// Case 2
		
		else if (PD_case == 3)
			RUDDER_PW = (long) PW_NEUT + (long) (20) * (long) (error) + (long) (70) *(long) (error - prev_error);	// Case 3
		
		else if (PD_case == 4)
			RUDDER_PW = (long) PW_NEUT + (long) (30) * (long) (error);	// Case 4
		
		prev_error = error;	// Store previous error
		
		// Stop car from steering too far to the right\
		if (RUDDER_PW > PW_MAX)
			RUDDER_PW = PW_MAX;
		
		// Stop car from steering too far to the left
		else if (RUDDER_PW < PW_MIN)
			RUDDER_PW = PW_MIN;
		
		new_heading = 0;	// Clear compass read flag
		
	}
	
	// Center rudder fan if Steer Slideswitch in STOP position or battery low
	if (STEER_SWITCH || battery <= 10)
		RUDDER_PW = PW_NEUT;
	
	PCA0CPL0 = 0xFFFF - RUDDER_PW;		// Set low byte of CCM0 --> rudder fan heading
	PCA0CPH0 = (0xFFFF - RUDDER_PW) >> 8;	// Set high byte of CCM0 --> rudder fan heading
	
}

// Drive Thrust Fans
void Drive_Thrust(void) {
	
	if (new_range) {
		
		range = Read_Ranger();	// Read Ranger
		
		// Determine thrust needed to reach desired altitude
		range_error = 50 - range;
		
		// Get DC Motor pulsewidth
		THRUST_PW = (long) PW_NEUT + (long) (kp_thrust) * (long) (range_error) + (long)(kd_thrust) * (long) (range_error - prev_range_error);
		
		prev_range_error = range_error;		// Store previous error
		
		// Stop blimp from driving too much in the upward direction
		if (THRUST_PW > PW_MAX)
			THRUST_PW = PW_MAX;
		
		// Stop blimp from driving too much in the downward direction
		else if (THRUST_PW < PW_MIN)
			THRUST_PW = PW_MIN;
		
		new_range = 0;	// Clear ranger read flag
		
	}
	
	// Stop thrust fans if Drive Slideswitch in STOP position
	if (DRIVE_SWITCH)
		THRUST_PW = PW_NEUT;
	
	// Set thrust fans down if battery low
	if (battery <= 10)
		THRUST_PW = PW_MIN;
	
	PCA0CPL2 = 0xFFFF - THRUST_PW;		// Set low byte of CCM2 --> left thrust fan power
	PCA0CPH2 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM2 --> left thrust fan power
	PCA0CPL3 = 0xFFFF - THRUST_PW;		// Set low byte of CCM3 --> right thrust fan power
	PCA0CPH3 = (0xFFFF - THRUST_PW) >> 8;	// Set high byte of CCM3 --> right thrust fan power
	
}

// Read Compass
int Read_Compass(void) {
	
	unsigned char addr = 0xC0;		// Address of compass sensor
	unsigned char Data[2];			// Data array of length 2
	int heading;				// Car heading in tenths of a degree
	i2c_read_data(addr, 2, Data, 2);	// Read two bytes from compass, start at register 2
	
	heading = (((int) Data[0] << 8) | Data[1]);	// Combine the two bytes read
	
	return heading;		// Return car heading
	
}

// Read Ranger
unsigned int Read_Ranger(void) {
	
	unsigned char addr = 0xE0;		// Address of ranger sensor
	unsigned char Data[2];			// Data array of length 2
	i2c_read_data(addr, 2, Data, 2);	// Read two bytes from ranger, start at register 2
	
	range = (((unsigned int) Data[0] << 8) | Data[1]);	// Combine the two bytes read
	
	Data[0] = 0x51;				// Start new ping
	i2c_write_data(addr, 0, Data, 1);	// Read one byte from ranger, start at register 0
	
	return range;	// Return ranger data
	
}

// Configure Blimp
void config_blimp(void) {
	
	lcd_clear();	// Clear display
	keypad_wait();	// Wait before reading user input from keypad
	
	lcd_print(" Adjust Thrust Fans\r");
	lcd_print(" 2 = Up\r");
	lcd_print(" 5 = Calibrate\r");
	lcd_print(" 8 = Down");
	
	keypad = '?';	// Clear keypad value
	
	// Wait for valid user input
	while (keypad != '5') {
		
		keypad = read_keypad();		// Get user input from keypad
		
		if (keypad == '2')
			ANGLE_PW = ANGLE_PW + 20;	// Increase the steering pulsewidth by 20
		
		else if (keypad == '8')
			ANGLE_PW = ANGLE_PW - 20;	// Decrease the steering pulsewidth by 20
		
		PCA0CPL1 = 0xFFFF - ANGLE_PW;		// Set low byte of CCM1 --> thrust fans angle
		PCA0CPH1 = (0xFFFF - ANGLE_PW) >> 8;	// Set high byte of CCM1 --> thrust fans angle
		
		keypad_wait();	// Wait before reading user input from keypad
		
	}
	
	keypad_wait();	// Wait before reading user input from keypad
	lcd_clear();	// Clear display
	
	lcd_print(" 2 = North\r");
	lcd_print("4 = West 6 = East\r");
	lcd_print(" 8 = South");
	
	keypad = '?';	// Clear keypad value
	
	// Wait for valid user input
	while (keypad != '2' && keypad != '4' && keypad != '6' && keypad != '8') {
		
		keypad = read_keypad();		// Get user input from keypad
		
		if (keypad == '2')
			desired_heading = 3600;		// North
		
		else if (keypad == '4')
			desired_heading = 2700;		// West
		
		else if (keypad == '6')
			desired_heading = 900;		// East
		
		else if (keypad == '8')
			desired_heading = 1800;		// South
		
		keypad_wait();	// Wait before reading user input from keypad
		
	}
	
	keypad_wait();	// Wait before reading user input from keypad
	lcd_clear();	// Clear display
	
	lcd_print(" Rudder Gain Kd/Kp\r\r");
	lcd_print("1: 70/.5 2: 100/2\r");
	lcd_print("3: 70/20 4: 0/20\r");
	
	keypad = '?';	// Clear keypad value
	
	// Wait for valid user input
	while (keypad != '1' && keypad != '2' && keypad != '3' && keypad != '4') {
		
		keypad = read_keypad();		// Get user input from keypad
		
		if (keypad == '1')
			PD_case = 1;	// Case 1
		
		else if (keypad == '2')
			PD_case = 2;	// Case 2
		
		else if (keypad == '3')
			PD_case = 3;	// Case 3
		
		else if (keypad == '4')
			PD_case = 4;	// Case 4
		
		keypad_wait();	// Wait before reading user input from keypad
		
	}
	
	keypad_wait();	// Wait before reading user input from keypad
	lcd_clear();	// Clear display
	
	lcd_print("Thrust Gain:5=exit\r");
	lcd_print(" 1: kd + 3: kd -\r");
	lcd_print(" 4: kp + 6: kp -\r");
	lcd_print(" kd=%u kp=%u.%u", kd_thrust, (int) kp_thrust, (int) ((kp_thrust - (int)kp_thrust) * 10));
	
	keypad = '?';	// Clear keypad value
	
	// Wait for valid user input
	while (keypad != '5') {
		
		keypad = read_keypad();		// Get user input from keypad
		
		if (read_keypad() == '1')
			kd_thrust = kd_thrust + 20;	// Increase derivative thrust gain by 20
		
		if (read_keypad() == '3')
			kd_thrust = kd_thrust - 20;	// Decrease derivative thrust gain by 20
		
		if (read_keypad() == '4')
			kp_thrust = kp_thrust + 0.3;	// Increase proportional thrust gain by 0.3
		
		if (read_keypad() == '6')
			kp_thrust = kp_thrust - 0.3;	// Decrease proportional thrust gain by 0.3
		
		if (keypad == '1' || keypad == '3' || keypad == '4' || keypad == '6') {
			
			lcd_clear();	// Clear display
			lcd_print("Thrust Gain:5=exit\r");
			lcd_print(" 1: kd + 3: kd -\r");
			lcd_print(" 4: kp + 6: kp -\r");
			lcd_print(" kd=%u kp=%u.%u", kd_thrust, (int) kp_thrust, (int) ((kp_thrust- (int) kp_thrust) * 10));
			
		}
		
		keypad_wait();	// Wait before reading user input from keypad
		
	}
	
	lcd_clear();	// Clear display
	
}

// Wait before reading user input from keypad
void keypad_wait(void) {
	
	wait = 0;	// Clear wait counter
	
	// Loop until wait counter overflows
	while (wait < 25534) {
		
		wait++;	// Increment wait counter
		printf("");
		
	}
	
}