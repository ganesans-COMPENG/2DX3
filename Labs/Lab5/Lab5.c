// COMPENG 2DX3 Studio 5 Code Template

//  Written by Ama Simons
//  January 30, 2020
//  Last Update: Feb 8, 2024 by Dr. Shahrukh Athar, 
//	This code provides a base upon which you should develop the two experiments in Studio 5

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "Systick.h"
#include "PLL.h"

void PortE_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		              // Activate the clock for Port E
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){};	      // Allow time for clock to stabilize
  
	GPIO_PORTE_DIR_R = 0b00000011;														// Enable PE0 and PE1 as outputs
	GPIO_PORTE_DEN_R = 0b00000011;                        		// Enable PE0 and PE1 as digital pins
	return;
	}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
		
	GPIO_PORTM_DIR_R = 0b00000000;       								      // Enable PM0 and PM1 as inputs 
  GPIO_PORTM_DEN_R = 0b00000011;														// Enable PM0 and PM1 as digital pins
	return;
}

//Enable LED D1, D2. Remember D1 is connected to PN1 and D2 is connected to PN0
void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};				// Allow time for clock to stabilize
		
	GPIO_PORTN_DIR_R=0b00000011;															// Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R=0b00000011;															// Enable PN0 and PN1 as digital pins
	return;
}

//Enable LED D3, D4. Remember D3 is connected to PF4 and D4 is connected to PF0
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};					// Allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R=0b00010001;															// Enable PF0 and PF4 as outputs
	GPIO_PORTF_DEN_R=0b00010001;															// Enable PF0 and PF4 as digital pins
	return;
}


int main(void){
	PortE_Init();
	PortM_Init();
	PortN_Init();
	PortF_Init();
	
	// Turn off LEDs at reset
	GPIO_PORTF_DATA_R = 0b00000000;
	GPIO_PORTN_DATA_R = 0b00000000;
	

	while(1){// Keep checking if a button is pressed 
	
		// Drive Low PE0 (Row 0) for scanning
		GPIO_PORTE_DATA_R =  0b11111110;
		
		// Check if Button # is pressed - D3 ON
		// Unique code is: 100 - In order of PM1 PM0 PE0
		while((GPIO_PORTM_DATA_R & 0b00000001)==0){
		GPIO_PORTF_DATA_R = 0b00010000;
		}
		// D3 OFF again
		GPIO_PORTF_DATA_R = 0b00000000;
		
		// Expand the above code to a 1x2 Keypad (Write your code below)
		// Check if Button D is pressed - If pressed turn LED D4 ON
		// Unique Code is (Fill in the blank): ______ - In order of PM1 PM0 PE0 
		
		
		//D4 OFF again
		while((GPIO_PORTM_DATA_R & 0b00000010)==0){
		GPIO_PORTF_DATA_R = 0b00000001;
		}
		// D4 OFF again
		GPIO_PORTF_DATA_R = 0b00000000;
		

		// ***************************************************
		// Now further expand the above code to a 2x2 Keypad
		// Write your code below
		// Don't forget to update the Unique codes in order of PM1 PM0 PE1 PE0
		// Drive Low PE1 (Row 0) for scanning
		GPIO_PORTE_DATA_R =  0b11111101;
		
		// Check if Button # is pressed - D3 ON
		// Unique code is: 100 - In order of PM1 PM0 PE0
		while((GPIO_PORTM_DATA_R & 0b00000001)==0){
		GPIO_PORTN_DATA_R = 0b00000010;
		}
		// D3 OFF again
		GPIO_PORTN_DATA_R = 0b00000000;
		
		// Expand the above code to a 1x2 Keypad (Write your code below)
		// Check if Button D is pressed - If pressed turn LED D4 ON
		// Unique Code is (Fill in the blank): ______ - In order of PM1 PM0 PE0 
		
		
		//D4 OFF again
		while((GPIO_PORTM_DATA_R & 0b00000010)==0){
		GPIO_PORTN_DATA_R = 0b00000001;
		}
		// D4 OFF again
		GPIO_PORTN_DATA_R = 0b00000000;
		
		
		
	}
}