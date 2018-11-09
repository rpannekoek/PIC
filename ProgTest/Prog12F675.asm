#include p12F675.inc

	radix dec ; Default radix: decimal

AnalogInput			EQU 0
KeyIn				EQU 1
KeyOut				EQU 2
Out1				EQU 4
Out2				EQU 5

; Comparator constants
CompModeOff			EQU 7

; ADC constants
AdcClockFoscDiv8	EQU 0x1 ; 500 kHz => Tadc = 22 us
AdcChannel0			EQU 0
AdcVrefVdd			EQU 0

; Timer 0 constants
T0ClockFoscDiv16	EQU 1 ; 250 kHz => T256 = 1024 us

; Timer 1 constants
T1ClockFoscDiv32	EQU 3 ; 125 kHz => T62500 = 500 ms

; RAM variables
	CBLOCK 0x20
		IsrSaveWREG
		IsrSaveSTATUS
		MoveCount
		LoopCountL
		LoopCountH
		LoopCountU
		T1InitL
		T1InitH
	ENDC

; =============================================================================
	ORG 0
Reset:
	goto Init

; =============================================================================
	ORG 4
Interrupt:
	; Save register state
	; Note: swapf is used as a trick to load WREG without affecting the STATUS register.
	movwf IsrSaveWREG
	swapf STATUS, w
	movwf IsrSaveSTATUS

	; Dispatch to appropriate Interrupt Service Routine
	btfsc INTCON, T0IF
	call Timer0ISR
	BANKSEL PIR1 ; 0
	btfsc PIR1, TMR1IF
	call Timer1ISR
	
	; Restore register state
	; Note: swapf is used as a trick to load WREG without affecting the STATUS register.
	swapf IsrSaveSTATUS, w
	movwf STATUS
	swapf IsrSaveWREG, f
	swapf IsrSaveWREG, w
	
	retfie

; =============================================================================
Timer0ISR:
	; Clear Timer 0 interrupt flag
	bcf INTCON, T0IF 

	; Pulse Out1 high
	BANKSEL GPIO ; 0
	bsf GPIO, Out1
	
	; Start new ADC and wait for result
	bsf ADCON0, GO_DONE 
WaitForADC:
	btfsc ADCON0, GO_DONE
	goto WaitForADC
	
	; Set Out1 low
	bcf GPIO, Out1

	; Use 1's complement of ADC result as T1 init.
	; So T1 interval is linear with ADC result.	
	BANKSEL ADRESH
	comf ADRESH, w
	movwf T1InitH
	BANKSEL ADRESL
	comf ADRESL, w
	movwf T1InitL
	
	return
	
; =============================================================================
Timer1ISR:
	; Clear Timer 1 interrupt flag
	BANKSEL PIR1 ; 0
	bcf PIR1, TMR1IF 

	; Reload Timer 1
	movfw T1InitL
	movwf TMR1L
	movfw T1InitH
	movwf TMR1H

	; Toggle Out2	
	BANKSEL GPIO ; 0
	movlw 1 << Out2
	xorwf GPIO, f

	return

; =============================================================================
CopyEEPROM2RAM:
	; FSR should contain the first EEPROM/RAM address to copy.
	; WREG should contain the number of bytes to copy.
	movwf MoveCount
	BANKSEL EEADR ; 1

CopyByte2RAM:
	movfw FSR
	movwf EEADR
	bsf EECON1, RD
	movfw EEDATA
	movwf INDF
	incf FSR, f
	decfsz MoveCount, f
	goto CopyByte2RAM

	BANKSEL GPIO ; 0
	return

; =============================================================================
CopyRAM2EEPROM:
	; FSR should contain the first RAM/EEPROM address to copy.
	; WREG should contain the number of bytes to copy.
	movwf MoveCount
	bcf INTCON, GIE; Prevent interrupts during EEPROM writing
	BANKSEL EECON1 ; 1
	bsf EECON1, WREN ; Enable EEPROM writes

CopyByte2EEPROM:
	BANKSEL PIR1; 0
	bcf PIR1, EEIF ; Clear EEPROM write complete interrupt flag
	BANKSEL EEADR; 1
	movfw FSR
	movwf EEADR
	movfw INDF
	movwf EEDATA
	movlw 0x55	; Write magic values to EECON2 to unlock a EEPROM write cycle
	movwf EECON2
	movlw 0xAA
	movwf EECON2
	bsf EECON1, WR
WriteWait:
	clrwdt
	BANKSEL PIR1; 0
	btfss PIR1, EEIF
	goto WriteWait	
	incf FSR, f
	decfsz MoveCount, f
	goto CopyByte2EEPROM

	BANKSEL EECON1 ; 1
	bcf EECON1, WREN ; Disable EEPROM writes
	bsf INTCON, GIE; Re-enable interrupts
	BANKSEL GPIO ; 0
	return
	
; =============================================================================
Init:
	BANKSEL GPIO ; 0
	movlw 1 << Out1 | 1 << Out2 | 1 << KeyOut
	movwf GPIO ; Turn on all outputs (diagnostic)

	; Configure digital I/O Ports
	movlw ~(1 << Out1 | 1 << Out2 | 1 << KeyOut)
	BANKSEL TRISIO ; 1
	movwf TRISIO ; Tristate the input ports

	movlw 1 << KeyIn
	BANKSEL WPU ; 1
	movwf WPU ; Configure Weak Pull-Up on Key input

	; Configure Comparator (off)
	movlw CompModeOff
	BANKSEL CMCON ; 0
	movwf CMCON

	; Configure A/D Convertor
	movlw 1 << AnalogInput | (AdcClockFoscDiv8 << ADCS0) 
	BANKSEL ANSEL ; 1
	movwf ANSEL ; Set analog input port and ADC clock
	movlw 1 << ADFM | AdcVrefVdd << VCFG | AdcChannel0 << CHS0 | 1 << ADON
	BANKSEL ADCON0 ; 0
	movwf ADCON0 ; Set ADC format, reference voltage, input channel and enable ADC

	; Calibrate INTOSC
	movlw 0x38 ; Original value in calibration memory (?)
	BANKSEL OSCCAL ; 1
	movwf OSCCAL

	; Output a 10 sec squarewave burst on Out1
	; Inner loop cycles: 5 => finner = 100 kHz (@fosc=4MHz)
	; Total loop cycles: ((5*255 + 7)*255 + 9) * U = 326919 * U
	; U=31 => 10 sec (@fosc=4MHz)
	clrf LoopCountL
	clrf LoopCountH
	movlw 31 ; 10 sec.
	movwf LoopCountU
	movlw 1 << Out1
	BANKSEL GPIO ; 0
Out1Burst:
	clrwdt
	xorwf GPIO, f
	decfsz LoopCountL, f
	goto Out1Burst
	decfsz LoopCountH, f
	goto Out1Burst
	decfsz LoopCountU, f
	goto Out1Burst

	; Configure Timer 0 as ADC time base (approx. 1 kHz)
	movlw T0ClockFoscDiv16
	BANKSEL OPTION_REG ; 1
	movwf OPTION_REG
	BANKSEL TMR0 ; 0
	clrf TMR0

	; Configure Timer 1 as low-frequency time base (2 Hz)
	movlw 0 << TMR1GE | T1ClockFoscDiv32 << T1CKPS0 | 0 << TMR1CS | 1 << TMR1ON
	BANKSEL T1CON ; 0
	movwf T1CON 
	clrf T1InitL
	clrf T1InitH
	clrf TMR1L
	clrf TMR1H

	; Enable interrupts
	movlw 1 << TMR1IE
	BANKSEL PIE1 ; 1
	movwf PIE1 ; Enable Timer 1 interrupts
	movlw 1 << GIE | 1 << PEIE | 1 << T0IE
	movwf INTCON ; Enable Timer 0 interrupts, peripheral interrupts (Timer 1) and interrupts in general

	BANKSEL GPIO ; 0

Wait4KeyDown:
	clrwdt
	btfsc GPIO, KeyIn
	goto Wait4KeyDown

	; Key down: set KeyOut Low and save T1Init to EEPROM
	bcf GPIO, KeyOut
	movlw T1InitL
	movwf FSR
	movlw 2 ; copy 2 bytes
	call CopyRAM2EEPROM
	
Wait4KeyUp:
	clrwdt
	btfss GPIO, KeyIn
	goto Wait4KeyUp
	
	; Key up: set KeyOut High
	bsf GPIO, KeyOut
	
	goto Wait4KeyDown
	

	END