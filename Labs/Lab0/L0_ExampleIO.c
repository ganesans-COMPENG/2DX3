//0. Documentation
//***************************************************************************
// Simple output C program based on the modification of the blinkLED.c code 
// provided by TI.
//
// There are 7 steps to initialize a GPIO port for general use
//		i. Activate the clock for the port by setting the corresponding bit in
//				the RCGCGPIO register and then wait for status bit in PRGPIO 
//				register to be true.
//	 ii. Unlock port if using PD7.
//	iii. Disable analog function of the pin. (Upon reset default = disabled)
//	 iv. Clear bits in PCTL to select regular digital function (see textbook 
//				tables 4.1&4.2, noting upon reset the default function is digital).
//		v. Set the data direction register (i.e., input or output).  In the DIR 
//				register, a bit set to: 0 = input, 1 = output.
//	 vi. Clear bits in the alternate function register (remember pins / ports 
//  			can have alternate functionality when configured as such.
//	vii. Enable the digtal port
//
// 	Note: Step i must be done first, but remaining steps may be performed in 
//		any order.  If your intention is to use a GPIO pin in its default 
//		digital state, then steps iii, iv, and vi can be omitted 
//		(also ii if not using PD7).
//
//	TEDoyle
//	October 26, 2019
 
//***************************************************************************
//1. Preprocessor Directives
//***************************************************************************
// There are library files that define the peripherals, ports, and pins.
// Review the msp432e401y.h file to become familiar with the symbolic labels 
// that have been preassigned to help simplify your programming. These labels
// can be different than your textbook, thus library familarity is important.

#include "msp.h"		// this libabry file chooses the approproate MSP library

//2. Functions
//***************************************************************************

// For now, no functions outside of main().

//3. Main
//***************************************************************************

int main(void)
{
	// step i.
	// Enable GPIO clocks for N & M peripheral
	SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R12;	// Port N (onboard LED)
	SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R11;	// Port M (GPIO for Lab 0 EPROM)

	// Check if the peripheral access is enabled.
	while(!(SYSCTL->PRGPIO & SYSCTL_PRGPIO_R12)); //N
	while(!(SYSCTL->PRGPIO & SYSCTL_PRGPIO_R11)); //M

	//skip steps ii, iii, iv
	
	// step v.
	/* Enable the GPIO pins for the onboard LEDs (PN1 = out, PN0 = out). */	
	GPION->DIR = 0x03;
	
	/* Enable the GPIO pins for the EPROM (PM7:0 = out). */
	GPIOM->DIR = 0xFF;
	
	//skip steps vi
	
	// step vii.
	GPION->DEN = 0x03;
	GPIOM->DEN = 0xFF;
	
	// main code
	uint32_t counter = 0;
	
	GPION->DATA = 0x01; 	//initialize the bit values

while(1)
	{
	    // Toggle GPIO pin value
	    GPION->DATA ^= BIT0;
			//GPION->DATA ^= BIT1;
	    for(counter = 0; counter < 1000000; counter++) {} //orig set to 200000
		  GPIOM->DATA = 0xAA;																//bits 0-7 of Port M
	}
}
