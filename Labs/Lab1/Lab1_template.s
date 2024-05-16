;Template - Defining an Assembly File 
;
; Original: Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu

; Modified by: Someshwar Ganesan, Abaan Khan
; January 24, 2023
; Program for Lab 1 Milestone 1


; McMaster University, Comp Eng 2DX3, Winter 2023
; Further resources: Valvano textbook, Chs. 3 & 4

;ADDRESS SETUP
;Define your I/O Port Addresses Here

SYSCTL_RCGCGPIO_R             EQU     0x400FE608         ;Step 1: GPIO Run Mode Clock Gating Control Register Address
GPIO_PORTN_DIR_R              EQU     0x40064400         ;Step 3: GPIO Port N DIR Register Address
GPIO_PORTN_DEN_R              EQU     0X4006451C         ;Step 4: GPIO Port N DEN Register Address
GPIO_PORTN_DATA_R             EQU     0x400643FC         ;Step 5: GPIO Port N DATA Register Address

                              



        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT Start

;Function PortN_Init
;Enable PortN and set bit PN1 for digital output
PortN_Init 
		;STEP 1 Activate clock (Set bit 5 in RCGCGPIO; In C pseudcode: SYSCTL_RCGCGPIO_R |= #0x20)
		 LDR R1, =SYSCTL_RCGCGPIO_R		;Stores the address of the RCGCGPIO register in R1
		 LDR R0, [R1]					;Dereferences R1 to put the contents of the RCGCGPIO register in R0
		 ORR R0,R0, #0x1000				;Modifies the contents of R0 to set bit 5 without changing other bits
		 STR R0, [R1]					;Stores the new value into the RCGCGPIO register
		
		;STEP 2: Wait for Peripheral Ready
		 NOP
		 NOP
		 
		
		;STEP 3: Set Port Direction (set bit 4 to 1 for output)
		 LDR R1, =GPIO_PORTN_DIR_R			;Load the memory address of the GPIODIR Port F Register into R1 (pointer)
		 LDR R0, [R1]           			;Load the contents from the memory address of GPIODIR Port F Register into R0
		 ORR R0,R0, #0x02			        ;Modify the contents of R0 to set bit 4 without changing other bits
		 STR R0, [R1]			            ;Store what is in R0 into address pointed by R1 
		 
		;STEP 4: Enable Digital Functioning (set bit 4 to 1 for digital enable)
		 LDR R1, =GPIO_PORTN_DEN_R			;Load the memory address of the GPIODEN Port F Register into R1 (pointer)
		 LDR R0, [R1]			        ;Load the contents from the memory address of GPIODEN Port F Register into R0
		 ORR R0,R0, #0x02			    ;Modify the contents of R0 to set bit 4 without changing other bits
		 STR R0, [R1]		        ;Store what is in R0 into address pointed by R1 

        BX LR               ; return from function 
		

		 

       
Start 
	    BL PortN_Init       ; call and execute PortF_Init
		
		
		;STEP 5: Outputting a Logic High to Port F Pin 4 (to switch on D3)   
		 LDR R1, =GPIO_PORTN_DATA_R			;Load the memory address of the GPIODATA Port F Register into R1 (pointer)
		 LDR R0, [R1]			            ;Load the contents from the memory address of GPIODATA Port F Register into R0
		 ORR R0,R0, #0x02			        ;Modify the contents of R0 to set bit 4 without changing other bits
		 
		 STR R0, [R1]			            ;Store what is in R0 into address pointed by R1 
		 
branchA
		 LDR R1, =GPIO_PORTN_DATA_R			;Load the memory address of the GPIODATA Port F Register into R1 (pointer)
		 LDR R0, [R1]			            ;Load the contents from the memory address of GPIODATA Port F Register into R0
		 			        ;Modify the contents of R0 to set bit 4 without changing other bits
		 EOR R0,R0, #0x02
		 STR R0, [R1]			            ;Store what is in R0 into address pointed by R1 
		 LDR R2, = 2000000
		 
delay
		 
		SUBS R2, R2, #1
		BNE delay
		B branchA
		 
		 
		 ;LDR R1, =GPIO_PORTN_DATA_R			;Load the memory address of the GPIODATA Port F Register into R1 (pointer)
		 ;LDR R0, [R1]			            ;Load the contents from the memory address of GPIODATA Port F Register into R0
		 			        ;Modify the contents of R0 to set bit 4 without changing other bits
		 ;AND R0,R0, #0xEF
		 ;STR R0, [R1]			            ;Store what is in R0 into address pointed by R1 
		 
		;BL LightON
		 
		 
		ALIGN               ; directive for assembly			
        END                 ; End of function 