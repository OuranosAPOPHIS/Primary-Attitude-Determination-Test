/*
 * Team Ouranos
 * Project: Aerial Platform for Overland Haul and Import System (APOPHIS)
 *
 *  Created On: March 7 , 2017
 *  Last Updated: March 8, 2017
 *  Test Date: March 8, 2017
 *      Author(s): Brandon Klefman
 *                 Damian Barraza
 *                 Abigail Couto
 *
 *      Purpose: Primarty attitude determination: GNC subsystem test.
 *
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"

#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "sensorlib/comp_dcm.h"

#include "misc/buttons.h"

#include "utils/uartstdio.h"

#include "initializations.h"
#include "APOPHIS_pin_map.h"
#include "packet_format.h"

#include "sensors/bmi160.h"
#include "sensors/i2c_driver.h"

//
// Sensor-Boosterpack defines.
// This allows alternation between the three
// different sensor packs and their associated
// calibration data. Only the first true value of
// the three defines will be processed, the others
// will be skipped. It is safe to leave one as true
// and the other two as false.
#define BOOSTERPACK1 false
#define BOOSTERPACK2 true
#define BOOSTERPACK3 false

//
// Sensor-Boosterpack Calibration data.
#if BOOSTERPACK1
#include "sensors/accel_gyro_cal_data1.h"
#elif BOOSTERPACK2
#include "sensors/accel_gyro_cal_data2.h"
#elif BOOSTERPACK3
#include "sensors/accel_gyro_cal_data3.h"
#endif

#include "attitude_estimation.h"

extern void CustomCompDCMUpdate(tCompDCM *psDCM);
extern void CustomCompDCMStart(tCompDCM *psDCM);


//*****************************************************************************
//
// Defines
//
//*****************************************************************************

#define DEBUG false
#define APOPHIS false

#define SPEEDIS120MHZ true

#define CALIB_TEST false
#define VALID_TEST true

#if CALIB_TEST
#define STABBIAS false
#else
#define STABBIAS true
#endif


//
// The number of LSB per degree/s for 125 degrees/s.
#define GYROLSB 262.4
#define ACCELLSB 16384

#define DT 0.01


//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void SysTickIntHandler(void);
void ConsoleIntHandler(void);
void RadioIntHandler(void);
void BMI160IntHandler(void);
void RadioTimeoutIntHandler(void);
void TurnOnLED(uint32_t LEDNum);
void TurnOffLED(uint32_t LEDNum);
void Menu(char CharReceived);
void ProcessRadio(void);
void SendPacket(void);
void ProcessIMUData(void);
void WaitForButtonPress(uint8_t ButtonState);

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//
// State of the system structure definition and variable.
typedef struct {
	bool bFlyOrDrive;   // Drive is false, fly is true.
	bool bMode;         // Autonomous is true, manual is false.
	bool bPayDeployed;  // True indicates the payload has already been deployed.
	bool bPayDeploying; // True indicates payload is being deployed.
	bool bRadioConnected; // True indicates radio is connected.
	bool bTargetSet;    // True indicates good radio data.
	float fRoll;		// Actual platform roll (degrees).
	float fPitch;		// Actual platform pitch (degrees).
	float fYaw;			// Actual platform yaw (degrees).
	float fCurrentLat;		// Current Latitide.
	float fCurrentLong;		// Current longitude.
	float fTargetLat;		// Target latitude.
	float fTargetLong;		// Target longiutde.
	float fTempTargetLat;// Temporary target latitude for before the location is set by the GS.
	float fTempTargetLong; // Temporary target longitude for before the location is set by the GS.
} SystemStatus;

//
// Global status structure.
SystemStatus sStatus;

//
// Radio packet to be sent to the ground station.
uTxPack g_Pack;

//
// Radio packet to be received from ground station.
uRxPack g_sRxPack;

//
// Counter for Systick printing at 2 Hz instead of 12 Hz.
uint32_t g_SysTickCount = 0;

//
// Variable to store when USER LED 4 is on.
bool g_LED4On = false;

//
// Variable to store the system clock speed.
uint32_t g_SysClockSpeed;

//
// Variable to store characters received from the PC.
char g_CharConsole;

//
// Variable to trigger evaluation of received character from PC in the main
// function.
bool g_ConsoleFlag = false;

//
// Variable to indicate end of program.
bool g_Quit = false;

//
// Variable to determine when to evaluate the ADC results.
bool g_ADCFlag = false;

//
// Variable to indicate when Radio data is available.
bool g_RadioFlag = false;

//
// Flag to indicate when to print for the trajectory information.
bool g_PrintFlag = false;

/*
 * Acceleromter and Gyro global values.
 */

//
// Flag to print raw data to console.
bool g_PrintRawBMIData = false;

//
// Comp DCM instance structure.
tCompDCM g_sCompDCMInst;
bool g_bDCMStarted;

//
// Float conversion for mag data.
float g_fMagLSB = 16;

//
// Offset compensation data for the accel and gyro.
uint8_t g_offsetData[7] = { 0 };

float g_GyroStabBias[3] = { 0 };

//
// Calculated bias for mag from MATLAB in uTeslas.
int16_t g_MagBias[3] = { 0 }; //{ 23.2604390452978, 4.40368720486817, 41.9678519105233 };

//
// Used as global storage for the gyro data.
float g_fGyroData[3];

//
// Global storage for accel data.
float g_fAccelData[3];

//
// Used as global storage for the raw mag data.
float g_fMagData[3];

//
// Used to indicate if the IMU is working, by blinking LED 1.
bool g_LED1On = false;

//
// Used to indicate when to print the accel and gyro values to the PC.
// Set to once per second. Triggered every second by the SysTickIntHandler()
bool g_loopCount = false;

//
// Floating point conversion factors for accelerometer. Found by dividing
// 9.81 m/s^2 by the LSB/g / 2. e.g. 9.81 / 8192 = 0.00119750976
float g_Accel2GFactor = 0.00119750976;

float g_fEulerAngles[3] = { 0.0f };

//
// Variable to track the frequency of packet sends to GS.
int32_t g_RadioCount = 0;

//
// Global structure for attitude estimation.
sAttitudeData sAttData;

//
// Variable to track when to use static vs. dynamic case. 
bool g_FirstTime = true;

//*****************************************************************************
//
// Start of program.
//
//*****************************************************************************
int main(void) {

#if STABBIAS
	int numCalcs = 0;
	int index, j;
	float fSum[3] = { 0 };
	float fbias[3][50] = { 0 };
#endif

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPUEnable();
	FPULazyStackingEnable();

	//
	// Set the clocking to run at 120 MHz.
#if SPEEDIS120MHZ
	g_SysClockSpeed = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
	SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
#else
	g_SysClockSpeed = SysCtlClockFreqSet(SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ, 16000000);
#endif

	//
	// Disable interrupts during initialization period.
	IntMasterDisable();

	//
	// Before doing anything, initialize the LED.
	InitLED(g_SysClockSpeed);

	//
	// Turn off all LEDs, in case one was left on.
	TurnOffLED(5);

	//
	// Initialization has begun. Turn on LED 1.
	TurnOnLED(1);

	//
	// Initialize the buttons.
	ButtonsInit();

	//
	// Initialize the Console if turned on.
	InitConsole();
	UARTprintf("Clock speed: %d\r\n", g_SysClockSpeed);

	//
	// Initialize the radio if turned on.
	InitRadio(g_SysClockSpeed);

	//
	// Initialize the BMI160 if enabled.
	InitIMU(g_SysClockSpeed, g_offsetData);

#if STABBIAS
	//
	// Calculate the gyro bias.
	while (numCalcs < 50) {
		uint8_t status;
		uint8_t IMUData[6] = { 0 };
		int16_t i16GyroData[3];
		float fGyroDataUnCal[3];

		//
		// First check the status for which data is ready.
		I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

		//
		// Check what status returned.
		if ((status & 0xC0) == (BMI160_GYR_RDY | BMI160_ACC_RDY)) {
			//
			// Then get the data for the gyro.
			I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 6, IMUData);

			//
			// Set the gyro data.
			i16GyroData[0] = (((int16_t) IMUData[1] << 8) + (int8_t) IMUData[0]);
			i16GyroData[1] = (((int16_t) IMUData[3] << 8) + (int8_t) IMUData[2]);
			i16GyroData[2] = (((int16_t) IMUData[5] << 8) + (int8_t) IMUData[4]);

			//
			// Convert gyro data to float.
			fGyroDataUnCal[0] = (((float) (i16GyroData[0])) / GYROLSB) - BGX;
			fGyroDataUnCal[1] = (((float) (i16GyroData[1])) / GYROLSB) - BGY;
			fGyroDataUnCal[2] = (((float) (i16GyroData[2])) / GYROLSB) - BGZ;

			//
			// Calculate the calibrated gyro data.
			fbias[0][numCalcs] = fGyroDataUnCal[0] * SGX + fGyroDataUnCal[1] * MGXY + fGyroDataUnCal[2] * MGXZ;
			fbias[1][numCalcs] = fGyroDataUnCal[0] * MGYX + fGyroDataUnCal[1] * SGY + fGyroDataUnCal[2] * MGYZ;
			fbias[2][numCalcs] = fGyroDataUnCal[0] * MGZX + fGyroDataUnCal[1] * MGZY + fGyroDataUnCal[2] * SGZ;

			numCalcs++;
		}
	}

	//
	// Calculate the bias.
	for (index = 0; index < numCalcs; index++)
		for (j = 0; j < 3; j++)
			fSum[j] += fbias[j][index];

	for (index = 0; index < 3; index++)
		g_GyroStabBias[index] = fSum[index] / numCalcs;
#endif

	//
	// Disable interrupts.
	IntMasterDisable();

	//
	// Initialize the state of the system.
	sStatus.bFlyOrDrive = false;
	sStatus.bMode = false;
	sStatus.bPayDeployed = false;
	sStatus.bRadioConnected = false;
	sStatus.bTargetSet = false;

	//
	// Set the magic packets.
	g_Pack.pack.Magic[0] = 0xFF;
	g_Pack.pack.Magic[1] = 0xFF;
	g_Pack.pack.Magic[2] = 0xFF;
	if (sStatus.bFlyOrDrive)
		g_Pack.pack.movement = 'D';
	else
		g_Pack.pack.movement = 'F';

	//
	// Set up the gyro weighting for the attitude estimation.
	InitAttitude(&sAttData, 0.75);

	//
	// Before starting program, wait for a button press on either switch.
	UARTprintf("Initialization Complete!\r\nPress left button to start.\r\n");

	TurnOnLED(5);

	WaitForButtonPress(LEFT_BUTTON);

	TurnOffLED(5);

	//
	// Initialize the DCM.
	//CompDCMInit(&g_sCompDCMInst, 1.0f / DCM_UPDATE_RATE, 0.2f, 0.6f, 0.2f);
	g_bDCMStarted = false;

	//
	// Initialization complete. Enable interrupts.
	IntMasterEnable();

	//
	// Turn off LED1, and enable the systick at 12 Hz to
	// blink LED 4, signifying regular operation.
	// The Systick cannot handle any value larger than 16MHz.
	TurnOffLED(1);
	SysTickPeriodSet(g_SysClockSpeed / 12);
	SysTickEnable();

	//
	// Activate the radio timers.
	TimerEnable(RADIO_TIMER, TIMER_A);
	TimerEnable(RADIO_TIMER_CHECK, TIMER_A);

	//
	// Enable the DCM.
	//if (g_bDCMStarted == 0)
	//	TimerEnable(DCM_TIMER, TIMER_A);

	//
	// Initialize the heading.
	InitHeading(&sAttData);

	//
	// Print menu.
	Menu('M');

	//
	// Program start.
	while (!g_Quit) {
		//
		// First check for commands from Console.
		if (g_ConsoleFlag)
			Menu(g_CharConsole);
	}
	//
	// Program ending. Do any clean up that's needed.
	UARTprintf("Goodbye!\r\n");

	I2CMasterDisable(BOOST_I2C);

	TurnOffLED(5);

	IntMasterDisable();

	return 0;
}

/*
 * Interrupt handlers go here:
 */
//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void SysTickIntHandler(void) {

	if (g_SysTickCount >= 5) {
		if (g_LED4On) {
			//
			// Turn off LED 4 if it is on.
			TurnOffLED(4);

			g_LED4On = false;
		} else {
			//
			// Otherwise turn it on.
			TurnOnLED(4);

			g_LED4On = true;
		}

		//
		// Trigger printing accel and gyro data to PC terminal.
		g_loopCount = true;

		//
		// Trigger printing of the trajectory information.
		g_PrintFlag = true;

		//
		// Reset SysTick Count.
		g_SysTickCount = 0;
	} else
		g_SysTickCount++;
}

//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void ConsoleIntHandler(void) {
	//
	// First get the interrupt status. Then clear the associated interrupt flag.
	uint32_t ui32Status = UARTIntStatus(CONSOLE_UART, true);
	UARTIntClear(CONSOLE_UART, ui32Status);

	//
	// Get the character sent from the PC.
	g_CharConsole = UARTCharGetNonBlocking(CONSOLE_UART);

	//
	// Echo back to Radio.
	//UARTCharPutNonBlocking(CONSOLE_UART, g_CharConsole);

	//
	// Trigger the flag for char received from console.
	g_ConsoleFlag = true;
}

//*****************************************************************************
//
// Interrupt handler which handles reception of characters from the radio.
//
//*****************************************************************************
void RadioIntHandler(void) {

	// TODO: wtf magic!?

	static uint8_t ui8Index = 0;
	static uint8_t ui8Magic[4] = { 0 };
	static uint8_t ui8MagicCount;
	static bool bValidData = false;
	int32_t i32RxChar;

	//
	// Get the interrupt status and clear the associated interrupt.
	uint32_t ui32Status = UARTIntStatus(RADIO_UART, true);
	UARTIntClear(RADIO_UART, ui32Status);

	//
	// Get the character received and send it to the console.
	while (UARTCharsAvail(RADIO_UART)) {
		i32RxChar = UARTCharGetNonBlocking(RADIO_UART);
		if (ui8Index >= (sizeof(uRxPack)))
			ui8Index = 0;
		if (i32RxChar != -1) {
			if (bValidData) {
				//
				// Get the chars over the UART.
				g_sRxPack.ui8Data[ui8Index++] = (uint8_t) i32RxChar;
				if (((g_sRxPack.ui8Data[3] == 'T' || g_sRxPack.ui8Data[3] == '0')
						&& ui8Index >= sizeof(tGSTPacket))
						|| (g_sRxPack.ui8Data[3] == 'C'
								&& ui8Index >= sizeof(tGSCPacket))) {
					ui8Index = 0;
					g_RadioFlag = true;
					bValidData = false;

					//
					// Good radio connection. Reset the timer and set the status.
					sStatus.bRadioConnected = true;
					TimerLoadSet(RADIO_TIMER_CHECK, TIMER_A,
							g_SysClockSpeed / 3);
					break;
				}
			} else {
				ui8Magic[ui8Index] = (uint8_t) i32RxChar;
				ui8Index = (ui8Index + 1) % 4;
				if (ui8MagicCount >= 3) {
					if (ui8Magic[ui8Index % 4] == 0xFF
							&& ui8Magic[(ui8Index + 1) % 4] == 0xFF
							&& ui8Magic[(ui8Index + 2) % 4] == 0xFF
							&& (ui8Magic[(ui8Index + 3) % 4] == 'T'
									|| ui8Magic[(ui8Index + 3) % 4] == 'C'
									|| ui8Magic[(ui8Index + 3) % 4] == '0')) {
						g_sRxPack.ui8Data[3] = ui8Magic[(ui8Index + 3) % 4];
						ui8Index = 4;
						bValidData = true;
						ui8MagicCount = 0;
					}
				} else {
					ui8MagicCount++;
				}
			}
		}
	}
}

//*****************************************************************************
//
// Interrupt handler for the BMI160 sensor unit.
//
//*****************************************************************************
void BMI160IntHandler(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = GPIOIntStatus(BOOST_GPIO_PORT_INT, true);

	//
	// Clear the interrupt.
	GPIOIntClear(BOOST_GPIO_PORT_INT, ui32Status);

	//
	// Check which interrupt fired.
	if (ui32Status == BOOST_GPIO_INT) {
		//
		// IMU data is ready.
	    ProcessIMUData();
	}
}

//*****************************************************************************
//
// Radio timeout interrupt. If this is reached, then we have lost
// communication.
//
//*****************************************************************************
void RadioTimeoutIntHandler(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(RADIO_TIMER_CHECK, true);

	//
	// Clear the interrupt.
	TimerIntClear(RADIO_TIMER_CHECK, ui32Status);

	//
	// Set the new status of the platform.
	sStatus.bRadioConnected = false;
	sStatus.bMode = true;
}

//*****************************************************************************
//
// Updates the DCM at a consistent rate of 25Hz.
//
//*****************************************************************************
void DCMUpdateTimer(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(DCM_TIMER, true);

	//
	// Clear the interrupt.
	TimerIntClear(DCM_TIMER, ui32Status);

	//
	// Check if this is the first time.
	if (g_bDCMStarted == 0) {
		//
		// Start the DCM.
		CompDCMAccelUpdate(&g_sCompDCMInst, g_fAccelData[0],
		                   g_fAccelData[1], g_fAccelData[2]);

		CompDCMGyroUpdate(&g_sCompDCMInst, g_fGyroData[0], g_fGyroData[1],
				g_fGyroData[2]);

		CompDCMMagnetoUpdate(&g_sCompDCMInst, g_fMagData[0], g_fMagData[1],
				g_fMagData[2]);

		CustomCompDCMStart(&g_sCompDCMInst);

		g_bDCMStarted = true;
	} else {
		//
		// DCM is already started, just update it.
		CompDCMAccelUpdate(&g_sCompDCMInst, g_Pack.pack.accelX,
				g_Pack.pack.accelY, g_Pack.pack.accelZ);

		CompDCMGyroUpdate(&g_sCompDCMInst, g_fGyroData[0], g_fGyroData[1],
				g_fGyroData[2]);

		CompDCMMagnetoUpdate(&g_sCompDCMInst, g_fMagData[0], g_fMagData[1],
				g_fMagData[2]);

		CustomCompDCMUpdate(&g_sCompDCMInst);
	}

	//
	// Get the Euler angles.
	CompDCMComputeEulers(&g_sCompDCMInst, &sStatus.fRoll, &sStatus.fPitch,
			&sStatus.fYaw);

	//
	// Flip the roll axis. Positive is roll right.
	sStatus.fRoll *= -1;

	//
	// Convert Eulers to degrees. 180/PI = 57.29...
	// Convert Yaw to 0 to 360 to approximate compass headings.
	sStatus.fRoll *= 57.295779513082320876798154814105f;
	sStatus.fPitch *= 57.295779513082320876798154814105f;
	sStatus.fYaw *= 57.295779513082320876798154814105f;
	if (sStatus.fYaw < 0)
		sStatus.fYaw += 360.0f;
}

/*
 * Other functions used by main.
 */
//*****************************************************************************
//
// This function will turn on the associated LED.
// Parameter: LEDNum - the desired LED number to turn on.
//
//*****************************************************************************
void TurnOnLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, LED1_PIN);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, LED2_PIN);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, LED3_PIN);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, LED4_PIN);
		return;
	}
	default: // Turn on all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, LED1_PIN | LED2_PIN);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, LED3_PIN | LED4_PIN);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will turn off the associated LED.
// Parameter: LEDNum - the desired LED number to turn off.
//
//*****************************************************************************
void TurnOffLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, 0x00);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, 0x00);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, 0x00);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, 0x00);
		return;
	}
	default: // Turn off all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, 0x00);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, 0x00);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will stop the current program run in order to wait for a
// button press.
//
// desiredButtonState: One of three values, LEFT_BUTTON, RIGHT_BUTTON or ALL_BUTTONS.
//
//*****************************************************************************
void WaitForButtonPress(uint8_t desiredButtonState) {
	uint8_t actualButtonState;
	uint8_t rawButtonState;
	uint8_t *pRawButtonState = &rawButtonState;
	uint8_t delta;
	uint8_t *pDelta = &delta;

	//
	// Get the state of the buttons.
	actualButtonState = ButtonsPoll(pDelta, pRawButtonState);

	if (desiredButtonState == LEFT_BUTTON) {
		while (actualButtonState != LEFT_BUTTON) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & LEFT_BUTTON;
		}
		return;
	} else if (desiredButtonState == RIGHT_BUTTON) {
		while (actualButtonState != RIGHT_BUTTON) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & RIGHT_BUTTON;
		}
		return;
	} else if (desiredButtonState == ALL_BUTTONS) {
		while (actualButtonState != ALL_BUTTONS) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & ALL_BUTTONS;
		}
		return;
	}
}

//*****************************************************************************
//
// This function will handle analysis of characters received from the console.
//
//*****************************************************************************
void Menu(char charReceived) {
	//
	// Check the character received.
	switch (charReceived) {
	case 'Q': // Quit the program
	{
		g_Quit = true;
		break;
	}

	case 'B': // Print raw accel, gyro and mag data.
	{
		if (g_PrintRawBMIData)
			g_PrintRawBMIData = false;
		else
			g_PrintRawBMIData = true;

		break;
	}
	case 'M': // Print Menu.
	{
		UARTprintf("Menu:\r\nM - Print this menu.\r\n");
		UARTprintf("B - Print raw accel, gyro and mag data.\r\n");
		UARTprintf("Q - Quit this program.\r\n");
		break;
	}
    }

	//
	// Reset the flag.
	g_ConsoleFlag = false;
}

//*****************************************************************************
//
// This function will forward all radio data to the console.
//
//*****************************************************************************
void ProcessRadio(void) {
	switch (g_sRxPack.ui8Data[3]) {
	case 'T': {
		//
		// Change the status of the platform.
		sStatus.bMode = true;

		//
		// The target location has now been set.
		sStatus.bTargetSet = true;

		break;
	}
	case 'C': {
		//
		// Change the status of the platform.
		sStatus.bMode = false;

		//
		// Check if we are flying or driving and update the Status.
		if (g_sRxPack.sControlPacket.flyordrive
				== g_sRxPack.sControlPacket.fdConfirm)
			if (g_sRxPack.sControlPacket.flyordrive == 'D')
				sStatus.bFlyOrDrive = false;
			else if (g_sRxPack.sControlPacket.flyordrive == 'F')
				sStatus.bFlyOrDrive = true;

		//
		// Check if we should deploy the payload.
		if ((!sStatus.bPayDeployed)
				&& (g_sRxPack.sControlPacket.payloadRelease
						== g_sRxPack.sControlPacket.prConfirm))
			if ((g_sRxPack.sControlPacket.payloadRelease == 1)
					&& (!sStatus.bPayDeploying)) {
				sStatus.bPayDeploying = true;
				    //ActivateSolenoids();
			}

		break;
	}
	case '0': {
		//
		// Change the mode to autonomous.
		sStatus.bMode = true;

		//
		// Receiving bad data. Tell main to ignore it.
		sStatus.bTargetSet = false;

		break;
	}
	}

	//
	// Reset the connection lost timer.
	// TODO: Set up a connection lost timeout timer.

	//
	// Reset the flag.
	g_RadioFlag = false;
}

//*****************************************************************************
//
// This function will send a packet to the ground station, if the radio is
// connected, it will be called by Timer 0 - Timer A at the rate specified.
//
//*****************************************************************************
void SendPacket(void) {

	int n;
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(RADIO_TIMER, true);

	//
	// Clear the interrupt.
	TimerIntClear(RADIO_TIMER, ui32Status);

	if (sStatus.bRadioConnected) {

        //
        // Assign these values to the
        g_Pack.pack.accelX = g_fAccelData[0];
        g_Pack.pack.accelY = g_fAccelData[1];
        g_Pack.pack.accelZ = g_fAccelData[2];
		g_Pack.pack.velX = g_fGyroData[0];
		g_Pack.pack.velY = g_fGyroData[1];
		g_Pack.pack.velZ = g_fGyroData[2];
		g_Pack.pack.posX = g_fMagData[0];
		g_Pack.pack.posY = g_fMagData[1];
		g_Pack.pack.posZ = g_fMagData[2];

		//
		// Current orientation.
		g_Pack.pack.roll = sStatus.fRoll;
		g_Pack.pack.pitch = sStatus.fPitch;
		g_Pack.pack.yaw = sStatus.fYaw;

		//
		// Mode of operation.
		if (sStatus.bFlyOrDrive) {
			g_Pack.pack.movement = 'F';
			g_Pack.pack.amtr1 = true;
			g_Pack.pack.amtr2 = true;
			g_Pack.pack.amtr3 = true;
			g_Pack.pack.amtr4 = true;
			g_Pack.pack.gndmtr1 = false;
			g_Pack.pack.gndmtr2 = false;
		} else {
			g_Pack.pack.movement = 'D';
			g_Pack.pack.gndmtr1 = true;
			g_Pack.pack.gndmtr2 = true;
			g_Pack.pack.amtr1 = false;
			g_Pack.pack.amtr2 = false;
			g_Pack.pack.amtr3 = false;
			g_Pack.pack.amtr4 = false;
		}

		//
		// Status bits.
		g_Pack.pack.uS1 = false;
		g_Pack.pack.uS2 = false;
		g_Pack.pack.uS3 = false;
		g_Pack.pack.uS4 = false;
		g_Pack.pack.uS5 = false;
		g_Pack.pack.uS6 = false;
		g_Pack.pack.payBay = sStatus.bPayDeployed;

		//
		// Send the data over the radio.
		for (n = 0; n < sizeof(g_Pack.str); n++)
			UARTCharPut(RADIO_UART, g_Pack.str[n]);
	}
}

//*****************************************************************************
//
// This function will retrieve the accel, gyro and mag data from the BMI160.
//
//*****************************************************************************
void ProcessIMUData(void) {
	uint8_t status;
	uint8_t IMUData[20]; // raw accel, gyro and mag data.
	int16_t i16AccelData[3];
	int16_t i16GyroData[3];
	int16_t i8MagData[3];
	float fAccelDataUnCal[3];
	float fGyroDataUnCal[3];

	//
	// First check the status for which data is ready.
	I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

	//
	// Check if the magnetometer data is ready.
	if ((status & 0x20) == (BMI160_MAG_RDY)) {
		//
		// Then get the data for both the accel, gyro and mag
		I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_MAG_X, 6, IMUData);

		//
		// Get the mag data.
		i8MagData[0] = (((int16_t) IMUData[1] << 8)
				+ (int8_t) (IMUData[0] & 0x1f));
		i8MagData[1] = (((int16_t) IMUData[3] << 8)
				+ (int8_t) (IMUData[2] & 0x1f));
		i8MagData[2] = (((int16_t) IMUData[5] << 8)
				+ (int8_t) (IMUData[4] & 0x7f));

		//
		// Convert to float for DCM and convert to teslas.
		g_fMagData[0] = ((i8MagData[0] / g_fMagLSB) - g_MagBias[0]) / 1e6;
		g_fMagData[1] = ((i8MagData[1] / g_fMagLSB) - g_MagBias[1]) / 1e6;
		g_fMagData[2] = ((i8MagData[2] / g_fMagLSB) - g_MagBias[2]) / 1e6;

		//
		// Update the mag data in the struct.
		UpdateMag(&sAttData, g_fMagData[0], g_fMagData[1], g_fMagData[2]);

		//
		// Blink the LED 1 to indicate sensor is working.
		if (g_LED1On) {
			TurnOffLED(1);
			g_LED1On = false;
		} else {
			TurnOnLED(1);
			g_LED1On = true;
		}
	}

	//
	// Check if the accel and gyro data are ready.
	if ((status & 0xC0) == (BMI160_ACC_RDY | BMI160_GYR_RDY)) {
		//
		// Then get the data for both the accel, gyro and mag
		I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 12, IMUData);

		//
		// Set the gyro data.
		i16GyroData[0] = (((int16_t) IMUData[1] << 8) + (int8_t) IMUData[0]);
		i16GyroData[1] = (((int16_t) IMUData[3] << 8) + (int8_t) IMUData[2]);
		i16GyroData[2] = (((int16_t) IMUData[5] << 8) + (int8_t) IMUData[4]);

		//
		// Set the accelerometer data.
		i16AccelData[0] = (((int16_t) IMUData[7] << 8) + (int8_t) IMUData[6]);
		i16AccelData[1] = (((int16_t) IMUData[9] << 8) + (int8_t) IMUData[8]);
		i16AccelData[2] = (((int16_t) IMUData[11] << 8) + (int8_t) IMUData[10]);

		//
		// Convert gyro data to float.
		fGyroDataUnCal[0] = (((float) (i16GyroData[0])) / GYROLSB) - BGX;
		fGyroDataUnCal[1] = (((float) (i16GyroData[1])) / GYROLSB) - BGY;
		fGyroDataUnCal[2] = (((float) (i16GyroData[2])) / GYROLSB) - BGZ;

		//
		// Compute the accel data to float.
		fAccelDataUnCal[0] = (((float) i16AccelData[0]) / ACCELLSB) - BAX;
		fAccelDataUnCal[1] = (((float) i16AccelData[1]) / ACCELLSB) - BAY;
		fAccelDataUnCal[2] = (((float) i16AccelData[2]) / ACCELLSB) - BAZ;

#if !CALIB_TEST
		//
		// Calculate the calibrated gyro data.
		g_fGyroData[0] = (fGyroDataUnCal[0] * SGX + fGyroDataUnCal[1] * MGXY + fGyroDataUnCal[2] * MGXZ) - g_GyroStabBias[0];
		g_fGyroData[1] = (fGyroDataUnCal[0] * MGYX + fGyroDataUnCal[1] * SGY + fGyroDataUnCal[2] * MGYZ) - g_GyroStabBias[1];
		g_fGyroData[2] = (fGyroDataUnCal[0] * MGZX + fGyroDataUnCal[1] * MGZY + fGyroDataUnCal[2] * SGZ) - g_GyroStabBias[2];

		//
		// Calculate the calibrated accelerometer data.
		g_fAccelData[0] = fAccelDataUnCal[0] * SAX + fAccelDataUnCal[1] * MAXY + fAccelDataUnCal[2] * MAXZ;
		g_fAccelData[1] = fAccelDataUnCal[0] * MAYX + fAccelDataUnCal[1] * SAY + fAccelDataUnCal[2] * MAYZ;
		g_fAccelData[2] = fAccelDataUnCal[0] * MAZX + fAccelDataUnCal[1] * MAZY + fAccelDataUnCal[2] * SAZ;
#endif
#if CALIB_TEST
		//
        // Just use the uncalibrated data.
        g_fGyroData[0] = fGyroDataUnCal[0];
        g_fGyroData[1] = fGyroDataUnCal[1];
        g_fGyroData[2] = fGyroDataUnCal[2];

        //
        // Just use the uncalibrated data.
        g_fAccelData[0] = fAccelDataUnCal[0];
        g_fAccelData[1] = fAccelDataUnCal[1];
        g_fAccelData[2] = fAccelDataUnCal[2];
#endif

		//
		// Update structure data.
		UpdateAccel(&sAttData, g_fAccelData[0], g_fAccelData[1], g_fAccelData[2]);
		UpdateGyro(&sAttData, g_fGyroData[0], g_fGyroData[1], g_fGyroData[2]);

		if (g_FirstTime) {
            //
            // static update.
            StaticUpdateAttitude(&sAttData);

#if CALIB_TEST || VALID_TEST
            //
            // maintain static attitude function.
            g_FirstTime = true;
#else
            //
            // Otherwise normal operation between static and dynamic.
            g_FirstTime = false;
#endif
		}
		else {
	        //
	        // dynamic update.
	        DynamicUpdateAttitude(&sAttData);
		}

		//
		// Update the Euler Angles.
		UpdateEulers(&sAttData);

		//
		// Assign the values.
		sStatus.fRoll = sAttData.fRoll;
		sStatus.fPitch = sAttData.fPitch;
		sStatus.fYaw = sAttData.fYaw;
	}

	if (g_PrintRawBMIData && g_PrintFlag)
	{
		int temp[3] = {(int)sAttData.fPitch, (int)sAttData.fRoll, (int)sAttData.fYaw};
		int dec[3] = {(int)(sAttData.fPitch * 100) - (int)(sAttData.fPitch),
					  (int)(sAttData.fRoll * 100) - (int)(sAttData.fRoll),
					  (int)(sAttData.fYaw * 100) - (int)(sAttData.fYaw)};

		UARTprintf("Pitch: %d.%d\r\n", temp[0], dec[0]);
		UARTprintf("Roll: %d.%d\r\n", temp[1], dec[1]);
		UARTprintf("Yaw: %d.%d\r\n", temp[2], dec[2]);

		//
		//UARTprintf("Gyro: %d, %d, %d\r\n", i16GyroData[0], i16GyroData[1], i16GyroData[2]);
	}

	//
	// Reset printing loop count for debugging.
    g_PrintFlag = false;

}
