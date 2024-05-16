; 2024_studio2_template - file for studio 2 exercises 
;
; Original: Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
;
; Modified by: Yaser M. Haddara
; Last modified: January 26, 2024
; Program for studio 2
; Initial template: implement a combinational lock using FSM
; Goal: implement a sequential lock using FSM
;
; McMaster University, Comp Eng 2DX3, Winter 2024
; Further resources: Valvano textbook, Chs. 3 & 4
;

;ADDRESS SETUP
;Define your I/O Port Addresses Here

SYSCTL_RCGCGPIO_R       EQU		0x400FE608  ;GPIO Run Mode Clock Gating Control Register Address

GPIO_PORTN_DIR_R		EQU 	0x40064400  ;GPIO Port N Direction Register address 
GPIO_PORTN_DEN_R        EQU 	0x4006451C  ;GPIO Port N Digital Enable Register address
GPIO_PORTN_DATA_R       EQU 	0x400643FC  ;GPIO Port N Data Register address
	
GPIO_PORTM_DIR_R        EQU		0x40063400  ;GPIO Port M Direction Register Address (Fill in these addresses)
GPIO_PORTM_DEN_R        EQU		0x4006351C  ;GPIO Port M Direction Register Address (Fill in these addresses)
GPIO_PORTM_DATA_R       EQU		0x400633FC  ;GPIO Port M Data Register Address      (Fill in these addresses) 
	
GPIO_PORTF_DIR_R        EQU		0x4005D400  ;GPIO Port M Direction Register Address (Fill in these addresses)
GPIO_PORTF_DEN_R        EQU		0x4005D51C  ;GPIO Port M Direction Register Address (Fill in these addresses)
GPIO_PORTF_DATA_R       EQU		0x4005D3FC 
	
;Define constants

COMBINATION			EQU		2_1010
COMBINATION_LENGTH	EQU		4
CLOCK_BIT			EQU		2_100	; clock is on PM2 
RESET_STATE			EQU		0                           



        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT Start

;Function PortM_Init
;Enable Port M and set bits PM0-PM2 for digital input
PortM_Init 
		;STEP 1 Activate clock (Set bit 5 in RCGCGPIO; In C pseudcode: SYSCTL_RCGCGPIO_R |= #0x20)
		 LDR R1, =SYSCTL_RCGCGPIO_R		;Stores the address of the RCGCGPIO register in R1
		 LDR R0, [R1]					;Dereferences R1 to put the contents of the RCGCGPIO register in R0
		 ORR R0,R0, #0x800				;Modifies the contents of R0 to set bit 11 without changing other bits
		 STR R0, [R1]					;Stores the new value into the RCGCGPIO register
		
		;STEP 2: Wait for Peripheral Ready
		 NOP
		 NOP
		 
		
		;STEP 3: Set Port Direction (set bit 0 to 0 for input)
		LDR R1, =GPIO_PORTM_DIR_R		;Load the memory address of the GPIODIR Port M Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODIR Port M Register into R0
		BIC R0, R0, #0x05				;Modify the contents of R0 to clear bits PM0-PM2 without changing other bits
		STR R0, [R1]					;Store what is in R0 into address pointed by R1 
		 
		;STEP 4: Enable Digital Functioning (set bit 0 to 1 for digital enable)
		LDR R1, =GPIO_PORTM_DEN_R		;Load the memory address of the GPIODEN Port M Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODEN Port M Register into R0
		ORR R0, R0, #0x05				;Modify the contents of R0 to set bits PM0-PM2 without changing other bits
		STR R0, [R1]					;Store what is in R0 into address pointed by R1 

        BX LR               ; return from function 

;Function PortN_Init
;Enable PortN and set bits PN0 and PN1 for digital output
PortN_Init 
		;STEP 1 Activate clock (Set bit 5 in RCGCGPIO; In C pseudcode: SYSCTL_RCGCGPIO_R |= #0x20)
		 LDR R1, =SYSCTL_RCGCGPIO_R		;Stores the address of the RCGCGPIO register in R1
		 LDR R0, [R1]					;Dereferences R1 to put the contents of the RCGCGPIO register in R0
		 ORR R0,R0, #0x1000				;Modifies the contents of R0 to set bit 12 without changing other bits
		 STR R0, [R1]					;Stores the new value into the RCGCGPIO register
		
		;STEP 2: Wait for Peripheral Ready
		 NOP
		 NOP
		 
		
		;STEP 3: Set Port Direction (set bit 4 to 1 for output)
		LDR R1, =GPIO_PORTN_DIR_R		;Load the memory address of the GPIODIR Port N Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODIR Port N Register into R0
		ORR R0, R0, #0x3				;Modify the contents of R0 to set bits PN0-PN1 without changing other bits
		STR R0, [R1]					;Store what is in R0 into address pointed by R1 
		 
		;STEP 4: Enable Digital Functioning (set bit 4 to 1 for digital enable)
		LDR R1, =GPIO_PORTN_DEN_R		;Load the memory address of the GPIODEN Port N Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODEN Port N Register into R0
		ORR R0, R0, #0x3				;Modify the contents of R0 to set bits PN0-PN1 without changing other bits
		STR R0, [R1]					;Store what is in R0 into address pointed by R1 

        BX LR               ; return from function 
       
;functions to detect the clock input

WaitForClockHigh 
		LDR R1, =GPIO_PORTM_DATA_R		;Load the memory address of the GPIODATA Port M Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODATA Port M Register into R0
		AND R7, R0,#0x04
		CMP R7, #CLOCK_BIT		;The 'S' modifier updates the flags. If result is 0, Z == 1 and condition EQ is true.
		BEQ WaitForClockHigh			;If it is 0, keep waiting

		LDR R7, =1200000
Debounce
		SUBS R7, #1
		BNE Debounce
		
		BX LR

WaitForClockLow 
		LDR R1, =GPIO_PORTM_DATA_R		;Load the memory address of the GPIODATA Port M Register into R1 (pointer)
		LDR R0, [R1]					;Load the contents from the memory address of GPIODATA Port M Register into R0
		AND R7, R0,#0x04
		CMP R7, #CLOCK_BIT			;Stores result in R2 to avoid changing R0. Result is 0 iff the clock bit is 0.
										;The 'S' modifier updates the flags. If result is 0, Z == 1 and condition EQ is true.
		BNE WaitForClockLow				;If it is not 0, keep waiting
		BX LR


Start 
	    BL PortM_Init       ; call and execute PortF_Init
		BL PortN_Init		; call and execute PortL_Init
		
Locked_State				; we start in the Locked_State
		LDR R1, =GPIO_PORTN_DATA_R	
		
		LDR R0, [R1]	
		
		BIC R0, R0, #0x01				;Set bit 0 - PN0 controls D2, ON for Locked
		ORR R0, R0, #0x02
							

		BL WaitForClockLow
		BL WaitForClockHigh		;When we return from this call, bit 0 of R0 is the input received
		
		
		LDR R2, =COMBINATION
		LDR R3, =COMBINATION_LENGTH
		SUB R3, #1
		LSR R2, R3
		AND R0, #0x1 
		CMP R0, R2
		BNE Locked_State

First_Digit					; if we get here, the correct first digit in the combination has been input
							; the state of the LEDs is still indicating LOCKED so no need to change them
	
		
		
		
		BL WaitForClockLow
		BL WaitForClockHigh			;When we return from this call, bit 0 of R0 is the input received
		
		
		LDR R2, =COMBINATION
		LDR R3, =COMBINATION_LENGTH
		SUB R3, #2
		LSR R2, R3
		AND R4, R2, #0x1
		AND R0, #0x1 
		CMP R0, R4
		BNE Locked_State
Second_Digit

		BL WaitForClockLow
		BL WaitForClockHigh				;When we return from this call, bit 0 of R0 is the input received
		
		
		LDR R2, =COMBINATION
		LDR R3, =COMBINATION_LENGTH
		SUB R3, #3
		LSR R2, R3
		AND R4, R2, #0x1
		AND R0, #0x1 
		CMP R0, R4
		BNE Locked_State

Third_Digit

		BL WaitForClockLow
		BL WaitForClockHigh
		
		LDR R1, =GPIO_PORTM_DATA_R		
		LDR R0, [R1]
		LDR R2, =COMBINATION
		AND R2, #0x1
		AND R0, #0x1
		CMP R0, R2
		BNE Locked_State		
		
Unlocked_State				; if we get here, the correct sequential combination has been input
		LDR R1, =GPIO_PORTN_DATA_R		
		LDR R0, [R1]					
		BIC R0, R0, #0x02				;Clear bit 0 - PN0 controls D2, OF for Unlocked
		ORR R0, R0, #0x01				;Set bit 1 - PN1 controls D1, ON for Unlocked
		STR R0, [R1]					
		
		BL WaitForClockLow
		BL WaitForClockHigh
		
		B Locked_State

		ALIGN               ; directive for assembly			
        END                 ; End of function
