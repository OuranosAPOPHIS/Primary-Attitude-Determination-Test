/*
 * initializations.c
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function definitions for all initializations of all peripherals.
 *      This will aid in simplicity for the main function.
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "APOPHIS_pin_map.h"
#include "initializations.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "sensors/bmi160.h"
#include "sensors/i2c_driver.h"

#define APOPHIS false

//*****************************************************************************
//
// External Function Declarations for IntRegister() functions.
//
//*****************************************************************************
extern void SysTickIntHandler(void);
extern void ConsoleIntHandler(void);
extern void RadioIntHandler(void);
extern void BMI160IntHandler(void);
extern void SendPacket(void);
extern void RadioTimeoutIntHandler(void);
extern void DCMUpdateTimer(void);

/*
 * LED Initialization function.
 */
//*****************************************************************************
//
// This function will initialize the 4 User LED pins. They are pins PN0, PN1
// PF0, and PF4.
//
//*****************************************************************************
void InitLED(uint32_t SysClockSpeed) {
	//
	// Initialize the GPIO port for the LEDs.
	SysCtlPeripheralEnable(LED_GPIO_PERIPH1);
	SysCtlPeripheralEnable(LED_GPIO_PERIPH2);

	//
	// Configure the pins as output pins.
	GPIOPinTypeGPIOOutput(LED_PORT1, LED1_PIN | LED2_PIN);
	GPIOPinTypeGPIOOutput(LED_PORT2, LED3_PIN | LED4_PIN);

	//
	// Initialize a 1 second SysTick for blinking the LED pin 4 to indicate
	// program running.
	SysTickPeriodSet(SysClockSpeed);

	//
	// Register the interrupt handler for blinking the LED and enable it.
	SysTickIntRegister(SysTickIntHandler);
	SysTickIntEnable();
}

/*
 * UART Initialization Functions
 */
//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the program is running.
//
//*****************************************************************************
void InitConsole(void) {
	//
	// Enable GPIO port A which is used for UART0 pins.
	SysCtlPeripheralEnable(CONSOLE_GPIO_PERIPH);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	GPIOPinConfigure(CONSOLE_CONFIG_PINRX);
	GPIOPinConfigure(CONSOLE_CONFIG_PINTX);

	//
	// Enable UART0 so that we can configure the clock.
	SysCtlPeripheralEnable(CONSOLE_PERIPH);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(CONSOLE_UART, UART_CLOCK_PIOSC);

	//
	// Select the alternate (UART) function for these pins.
	GPIOPinTypeUART(CONSOLE_PORT, CONSOLE_PINRX | CONSOLE_PINTX);

	//
	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);

	//
	// Enable the UART interrupt.
	IntEnable(CONSOLE_INT);
	UARTIntEnable(CONSOLE_UART, UART_INT_RX | UART_INT_RT);
	UARTIntRegister(CONSOLE_UART, ConsoleIntHandler);
}

//*****************************************************************************
//
// This function sets up UART RX/TX communication to enable communication with
// the 3D Robotics Radio V2.
//
//*****************************************************************************
void InitRadio(uint32_t SysClockSpeed) {
	//
	// Print initializing to the Console.
	UARTprintf("Initializing Radio...\n\r");

	//
	// Enable GPIO port K which is used for UART4 pins.
	SysCtlPeripheralEnable(RADIO_GPIO_PERIPH);

	//
	// Configure the pin muxing for UART4 functions on port K0 and K1.
	GPIOPinConfigure(RADIO_CONFIG_PINRX);
	GPIOPinConfigure(RADIO_CONFIG_PINTX);

	//
	// Enable UART4 so that we can configure the clock.
	SysCtlPeripheralEnable(RADIO_PERIPH);

	//
	// Select the alternate UART function for the Rx pin.
	GPIOPinTypeUART(RADIO_PORT, RADIO_PINRX | RADIO_PINTX);

	//
	// Configure UART6 for 57600, 8-N-1 operation.
	UARTConfigSetExpClk(RADIO_UART, SysClockSpeed, 57600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
			UART_CONFIG_PAR_NONE));

	//
	// Enable the UART interrupt.
	IntEnable(RADIO_INT);
	UARTIntEnable(RADIO_UART, UART_INT_RX | UART_INT_RT);
	UARTIntRegister(RADIO_UART, RadioIntHandler);

	//
	// Set up a timer, to detect when radio signal is lost.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//
	// Configure the timer for sending radio packets.
	TimerConfigure(RADIO_TIMER, TIMER_CFG_PERIODIC);
	TimerLoadSet(RADIO_TIMER, TIMER_A, SysClockSpeed / RADIO_TIMER_RATE);

	//
	// Configure the interrupts for the timer.
	TimerIntClear(RADIO_TIMER, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(RADIO_TIMER, TIMER_TIMA_TIMEOUT);
	IntEnable(RADIO_TIMER_INT);
	TimerIntRegister(RADIO_TIMER, TIMER_A, SendPacket);

	//
	// Configure the timer for radio timeout.
	// This will detect loss of radio communication.
	TimerConfigure(RADIO_TIMER_CHECK, TIMER_CFG_PERIODIC);
	TimerLoadSet(RADIO_TIMER_CHECK, TIMER_A, SysClockSpeed / GS_RADIO_RATE * 2);

	//
	// Configure the interrupts for the timer.
	TimerIntClear(RADIO_TIMER_CHECK, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(RADIO_TIMER_CHECK, TIMER_TIMA_TIMEOUT);
	IntEnable(RADIO_TIMER_CHECK_INT);
	TimerIntRegister(RADIO_TIMER_CHECK, TIMER_A, RadioTimeoutIntHandler);

	//
	// Initialization complete. Print to console.
	UARTprintf("Done!\n\r");
}

void InitIMU(uint32_t SysClockSpeed, uint8_t *offsetCompensation) {
	UARTprintf("Initializing IMU...\n\r");

	//
	// Stop the Clock, Reset and Enable I2C Module
	// in Master Function
	SysCtlPeripheralDisable(BOOST_PERIPH);
	SysCtlPeripheralReset(BOOST_PERIPH);
	SysCtlPeripheralEnable(BOOST_PERIPH);

	//
	// Wait for the Peripheral to be ready for programming
	while (!SysCtlPeripheralReady(BOOST_PERIPH))
		;

	//
	// Initialize the GPIO Peripherals used by this device.
	SysCtlPeripheralEnable(BOOST_GPIO_PERIPH1);
	SysCtlPeripheralEnable(BOOST_GPIO_PERIPH2);

	//
	// Initialize the I2C0 bus for configuration.
	SysCtlPeripheralEnable(BOOST_PERIPH);

	//
	// Configure the pins for GPIO I2C0 use.
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinTypeI2C(BOOST_GPIO_PORT_I2C, BOOST_GPIO_SDA);
	GPIOPinTypeI2CSCL(BOOST_GPIO_PORT_I2C, BOOST_GPIO_SCL);

	//
	// Configure the IMU interrupt on pin PC6.
	GPIOPinTypeGPIOInput(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT);
	GPIOIntTypeSet(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT, GPIO_RISING_EDGE);

	//
	// Enable and register the function for the interrupt.
	GPIOIntRegister(BOOST_GPIO_PORT_INT, BMI160IntHandler);
	GPIOIntEnable(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT);

	//
	// Disable the I2C module for configuration.
	I2CMasterDisable(BOOST_I2C);

	//
	// Configure and Initialize the I2C bus at 400 kpbs.
	I2CMasterInitExpClk(BOOST_I2C, SysClockSpeed, true);

	//
	// Enable the I2C module.
	I2CMasterEnable(BOOST_I2C);

	//
	// Since BME280 does not have interrupts,
	// configure timer to poll data.
	SysCtlPeripheralEnable(DCM_TIMER_PERIPH);

	//
	// Configure and enable the timer.
	//
	// Configure the timer to run at 25 Hz.
	TimerClockSourceSet(DCM_TIMER, TIMER_CLOCK_PIOSC);
	TimerConfigure(DCM_TIMER, TIMER_CFG_PERIODIC);
	TimerLoadSet(DCM_TIMER, TIMER_A, 16000000 / DCM_UPDATE_RATE);

	//
	// Configure the interrupts for the timer.
	TimerIntClear(DCM_TIMER, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(DCM_TIMER, TIMER_TIMA_TIMEOUT);
	IntEnable(DCM_TIMER_INT);
	TimerIntRegister(DCM_TIMER, TIMER_A, DCMUpdateTimer);

	//
	// Before calling the BMI160 initialize function, make sure the I2C
	// bus is not busy.
	while (I2CMasterBusBusy(BOOST_I2C))
		;

	//
	// Initialize the BMI160 to have a 100Hz update rate for the
	// accelerometer and gyro and 30 Hz for the magnetometer,
	// +/-2g setting on the accelerometer and 2000 deg/s for the gyro.
	InitBMI160(BOOST_I2C, BMI160_ACC_100_HZ, BMI160_ACC_RANGE_2G,
			BMI160_GYR_100_HZ,
			BMI160_GYR_RATE_2000, BMI160_MAG_31_HZ, offsetCompensation,
			SysClockSpeed);

	//
	// Turn off interrupts, since I2CWrite turns them on.
	IntMasterDisable();

	UARTprintf("Done!\n\r");
}
