#include p12F675.inc

	radix dec ; Default radix: decimal

; GPIO assignments
AnalogInput			EQU 0
MonitorLedGreen		EQU 1
MonitorLedRed		EQU 2
KeyInput			EQU 3
KeyLed				EQU 4
Relay				EQU 5

; Comparator constants
CompModeOff			EQU 7

; ADC constants
AdcClockFoscDiv8	EQU 0x1 ; 500 kHz => Tadc = 44 us
AdcChannel0			EQU 0
AdcVrefVdd			EQU 0

; Timer 0 constants
T0ClockFoscDiv16	EQU 1 ; 250 kHz => T256 = 1024 us

; Timer 1 constants
T1ClockFoscDiv32	EQU 3 ; 125 kHz => T62500 = 500 ms
T1Init				EQU 0x10000 - 62500
T1PostscalerInit	EQU 21 ; Tpostscaler = 10 s

; MonitorState bits
MonitorIdle			EQU 0
MonitorInlet		EQU 1
MonitorWashing		EQU 2
MonitorHeating		EQU 3
MonitorConfigure	EQU 4

; KeyState bits
KeyPressed			EQU 0
LastKeyState		EQU 1

SignBit				EQU 7

; RAM/EEPROM variables
	CBLOCK 0x20
		; RAM and EEPROM
		VzeroL
		VzeroH
		VidleL
		VidleH
		VinletL
		VinletH
		VwashingL
		VwashingH
		VheatingL
		VheatingH
		InletTimeout
		IdleTimeout

		; RAM only
		IsrSaveWREG
		IsrSaveSTATUS
		IsrSaveFSR
		T1Postscaler
		T1Countdown
		BlinkLEDs
		MoveCount
		VdiffL
		VdiffH
		VmonitorL
		VmonitorH
		VmonitorU
		MonitorFrameCount
		MonitorState
		KeyState
		SamplePtr
	ENDC

SampleBuffer		EQU 0x40

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
	movfw FSR
	movwf IsrSaveFSR

	; Dispatch to appropriate Interrupt Service Routine
	btfsc INTCON, T0IF
	call Timer0ISR
	BANKSEL PIR1 ; 0
	btfsc PIR1, TMR1IF
	call Timer1ISR
	
	; Restore register state
	; Note: swapf is used as a trick to load WREG without affecting the STATUS register.
	movfw IsrSaveFSR
	movwf FSR
	swapf IsrSaveSTATUS, w
	movwf STATUS
	swapf IsrSaveWREG, f
	swapf IsrSaveWREG, w
	
	retfie

; =============================================================================
Timer0ISR:
	; Clear Timer 0 interrupt flag
	bcf INTCON, T0IF 

	; Start new ADC and wait for result
	BANKSEL ADCON0 ; 0
	bsf ADCON0, GO_DONE 
WaitForADC:
	btfsc ADCON0, GO_DONE
	goto WaitForADC

	; Aggregate the abs. diff between ADC result and Vzero
	; Vdiff = ADC result - Vzero
	movfw VzeroH
	BANKSEL ADRESH
	subwf ADRESH, w
	movwf VdiffH
	movfw VzeroL
	BANKSEL ADRESL
	subwf ADRESL, w
	btfss STATUS, C
	decf VdiffH, f ; borrow from high-order byte
	movwf VdiffL

	; If Vdiff < 0 then Vdiff = -Vdiff
AbsVdiff:	
	btfss VdiffH, SignBit
	goto StoreVdiff
	comf VdiffH, f
	comf VdiffL, f
	movlw 1 ; 2's complement => +1
	addwf VdiffL, f
	addcf VdiffH, f

	; Store abs(Vdiff) in sample buffer (if enabled)
StoreVdiff:
	movfw SamplePtr
	bz Aggregate
	addlw SampleBuffer
	movwf FSR
	movfw VdiffL
	movwf INDF
	movfw SamplePtr
	addlw 1
	andlw 0x1F ; Wrap after 0x20 bytes
	movwf SamplePtr
	
Aggregate:
	; Note: we aggregate 10 bits samples into a 24 bit (signed) integer, giving us 13 bits "room".
	; So, up to 8k samples can be aggregated, but we use a frame size of 256 samples.
	; Afterwards, we dispose the low-order byte, giving us a 10 bits average.
	movfw VdiffH
	addwf VmonitorH, f
	addcf VmonitorU, f
	movfw VdiffL
	addwf VmonitorL, f
	bnc Aggregated
	incf VmonitorH, f ; carry to high-order byte
	btfsc STATUS, Z
	incf VmonitorU, f ; carry to upper-order byte	

Aggregated:
	; Return until monitor frame is complete
	decfsz MonitorFrameCount, f
	return

	; Monitor frame is complete.
	; In configuration mode: let the main routine pick up Vmonitor.
	btfsc MonitorState, MonitorConfigure
	return

	; Otherwise: determine the monitor state.
	movlw VidleH
	call DiffVmonitor
	movlw 1 << MonitorIdle
	btfsc VdiffH, SignBit
	goto SetMonitorState
	movlw VinletH
	call DiffVmonitor
	movlw 1 << MonitorInlet
	btfsc VdiffH, SignBit
	goto SetMonitorState
	movlw VwashingH
	call DiffVmonitor
	movlw 1 << MonitorWashing
	btfsc VdiffH, SignBit
	goto SetMonitorState
	movlw 1 << MonitorHeating

SetMonitorState:
	movwf MonitorState
	; Set monitor LEDs:
	; Green = inlet/washing
	; Red = washing/heating
	BANKSEL GPIO
	bcf GPIO, MonitorLedGreen  
	andlw 1 << MonitorInlet | 1 << MonitorWashing
	btfss STATUS, Z
	bsf GPIO, MonitorLedGreen  
	bcf GPIO, MonitorLedRed
	movfw MonitorState  
	andlw 1 << MonitorWashing | 1 << MonitorHeating
	btfss STATUS, Z
	bsf GPIO, MonitorLedRed 

	call InitMonitorFrame
	return
	
; =============================================================================
InitMonitorFrame:
	clrf VmonitorL
	clrf VmonitorH
	clrf VmonitorU
	; Monitor frame size = 256 (MonitorFrameCount will hit 0 again after 256 decrements).
	; With Tsample = 1.024 ms: Tframe = 262 ms (13 cycles @ 50 Hz).
	clrf MonitorFrameCount
	return

; =============================================================================
DiffVmonitor:
	; Input: WREG points to the high-order byte of the value to compare Vmonitor against.
	; Output: Vdiff = Vmonitor - value
	movwf FSR
	movfw INDF
	subwf VmonitorU, w
	movwf VdiffH
	decf FSR, f
	movfw INDF
	subwf VmonitorH, w
	btfss STATUS, C
	decf VdiffH, f ; borrow from high-order byte
	movwf VdiffL
	return

; =============================================================================
Timer1ISR:
	; Clear Timer 1 interrupt flag
	BANKSEL PIR1 ; 0
	bcf PIR1, TMR1IF 

	; Reload Timer 1
	movlw LOW T1Init
	movwf TMR1L
	movlw HIGH T1Init
	movwf TMR1H

	; Blink LEDs
	movfw BlinkLEDs
	xorwf GPIO, f

	; Determine keypress
	btfss GPIO, KeyInput
	goto KeyDown
KeyUp:
	bsf KeyState, LastKeyState
	goto Postscaler
KeyDown:
	btfsc KeyState, LastKeyState
	bsf KeyState, KeyPressed
	bcf KeyState, LastKeyState

Postscaler:
	; Decrement postscaler
	decfsz T1Postscaler, f
	return

	; Postscaler reached 0: reload and count down T1Countdown to 0.
	movlw T1PostscalerInit
	movwf T1Postscaler
	movlw 1
	subwf T1Countdown, w
	btfsc STATUS, C
	movwf T1Countdown
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
	clrf GPIO ; Turn off all outputs 

	; Configure digital I/O Ports
	movlw 1 << AnalogInput | 1 << KeyInput
	BANKSEL TRISIO ; 1
	movwf TRISIO ; Tristate the input ports

	; Configure Comparator (off)
	movlw CompModeOff
	BANKSEL CMCON ; 0
	movwf CMCON

	; Calibrate INTOSC
	;call 0x3FF
	movlw 0x40
	BANKSEL OSCCAL ; 1
	movwf OSCCAL

	; Configure A/D Convertor
	movlw 1 << AnalogInput | (AdcClockFoscDiv8 << ADCS0) 
	BANKSEL ANSEL ; 1
	movwf ANSEL ; Set analog input port and ADC clock
	movlw 1 << ADFM | AdcVrefVdd << VCFG | AdcChannel0 << CHS0 | 1 << ADON
	BANKSEL ADCON0 ; 0
	movwf ADCON0 ; Set ADC format, reference voltage, input channel and enable ADC

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
	movlw LOW T1Init
	movwf TMR1L
	movlw HIGH T1Init
	movwf TMR1H
	movlw T1PostscalerInit
	movwf T1Postscaler
	clrf BlinkLEDs
	clrf KeyState

	; Set voltage monitoring levels and timeouts (stored in EEPROM during Configuration)
	movlw VzeroL
	movwf FSR
	movlw IsrSaveWREG - VzeroL
	call CopyEEPROM2RAM

	; Initialize (Voltage) Monitor
	call InitMonitorFrame
	movlw 1 << MonitorIdle
	movwf MonitorState
	clrf SamplePtr

	; Enable interrupts
	movlw 1 << TMR1IE
	BANKSEL PIE1 ; 1
	movwf PIE1 ; Enable Timer 1 interrupts
	movlw 1 << GIE | 1 << PEIE | 1 << T0IE
	movwf INTCON ; Enable Timer 0 interrupts, peripheral interrupts (Timer 1) and interrupts in general

	BANKSEL GPIO ; 0
	
	; Wait 2 seconds for analog input to stabilize after power up.
	; Output squarewave burst on Key LED in the meantime.
	movlw 5
	movwf T1Postscaler
	movlw 1
	movwf T1Countdown
	clrw
StartupWait:
	clrwdt	
	xorlw 1 << KeyLed
	movwf GPIO
	tstf T1Countdown ; If timeout occurred goto Washing
	bnz StartupWait
	clrf GPIO	
	
	; If the key is down during startup, goto Configure.
	btfss GPIO, KeyInput
	goto Configure
	
; =============================================================================
Standby:
	; Let the Key LED blink and relay off.
	bsf BlinkLEDs, KeyLed
	bcf GPIO, Relay
StandbyWait:
	clrwdt
	btfsc KeyState, KeyPressed	
	goto Inlet1
	btfsc MonitorState, MonitorIdle ; If Vmon > idle goto Inlet
	goto StandbyWait
	
; =============================================================================
Inlet1:
	; We don't want to lose any time before activating the relay.
	; Maybe there was only a short spike that caused us to enter the inlet state,
	; but we'll evaluate that after a 10 seconds delay (in state Inlet2).
	bcf KeyState, KeyPressed
	; Turn the Key LED on and relay on.
	bcf BlinkLEDs, KeyLed
	bsf GPIO, KeyLed
	bsf GPIO, Relay
	; Set timeout: 10 s
	movlw 1
	movwf T1Countdown
Inlet1Wait:
	clrwdt	
	btfsc KeyState, KeyPressed	
	goto Washing
	tstf T1Countdown ; If timeout occurred goto Inlet2
	bnz Inlet1Wait	

; =============================================================================
Inlet2:
	; After a small delay (10 s), we check if the monitor voltage didn't fall back to idle level.
	; If it did, there probably was only a spike and we revert to standby state.
	btfsc MonitorState, MonitorIdle ; If Vmon = idle goto Standby
	goto Standby
	; Set configured timeout - 10 s
	decf InletTimeout, w
	movwf T1Countdown
Inlet2Wait:
	clrwdt	
	btfsc KeyState, KeyPressed	
	goto Washing
	tstf T1Countdown ; If timeout occurred goto Washing
	bnz Inlet2Wait	
	
; =============================================================================
Washing:
	bcf KeyState, KeyPressed
	; Turn the Key LED off and relay off.
	bcf GPIO, KeyLed
	bcf GPIO, Relay
WashingWait:
	clrwdt	
	btfsc KeyState, KeyPressed	
	goto Inlet1
	btfss MonitorState, MonitorIdle ; If Vmon = idle goto Idle
	goto WashingWait

; =============================================================================
Idle:
	; Key LED and relay should already be off.
	; Set timeout
	movfw IdleTimeout
	movwf T1Countdown
IdleWait:
	clrwdt	
	btfsc KeyState, KeyPressed	
	goto Inlet1
	btfss MonitorState, MonitorIdle ; If Vmon > idle goto Washing
	goto Washing	
	tstf T1Countdown ; If timeout occurred goto Standby
	bz Standby
	goto IdleWait

; =============================================================================
Configure:
	bsf MonitorState, MonitorConfigure
	clrf GPIO ; Turn off all outputs
	
	; Measure Vzero:
	; Set Vzero = 0 and measure Vmonitor.
	; New Vzero = Vmonitor / MonitorFrameSize
	clrf VzeroL
	clrf VzeroH
	movlw VzeroH
	call Measure
	
	; Measure Vidle
	movlw VidleH
	call Measure
	
	; Measure Vinlet (after key pressed)
	bsf BlinkLEDs, MonitorLedGreen ; Blink Green LED
	call WaitForKeyPress
	incf SamplePtr, f ; Start storage of samples in sample buffer
	movlw VinletH
	call Measure
	clrf SamplePtr ; Stop storage of samples in sample buffer

	; Copy the sample buffer to EEPROM	
	movlw SampleBuffer
	movwf FSR
	movlw 0x20
	call CopyRAM2EEPROM

	; Measure Vwashing (after key pressed)
	bsf BlinkLEDs, MonitorLedRed ; Blink Green + Red = Orange
	clrf GPIO ; Ensure Green and Red blink in sync
	call WaitForKeyPress
	movlw VwashingH
	call Measure
		
	; Measure Vheating (after key pressed)
	bcf BlinkLEDs, MonitorLedGreen ; Blink Orange - Green = Red
	clrf GPIO;
	call WaitForKeyPress
	movlw VheatingH
	call Measure

	clrf BlinkLEDs ; Stop blinking
	clrf GPIO;
	
	; Determine Vidle threshold: avg(Vidle,Vinlet)
	movlw VidleH
	call Average

	; Determine Vinlet threshold: avg(Vinlet,Vwashing)
	movlw VinletH
	call Average
	
	; Determine Vwashing threshold: avg(Vwashing,Vheating)
	movlw VwashingH
	call Average

	; TODO: Configure timeouts
	movlw 60 ; 10 minutes
	movwf InletTimeout
	movlw 30 ; 5 minutes
	movwf IdleTimeout

	; Copy the configured values to EEPROM	
	movlw VzeroL
	movwf FSR
	movlw IsrSaveWREG - VzeroL
	call CopyRAM2EEPROM
	
	bcf MonitorState, MonitorConfigure
	goto Standby

; =============================================================================
Measure:
	; Input: WREG points to high-order byte of value to measure
	movwf FSR
	call InitMonitorFrame ; Sets MonitorFrameCount to 0.
MeasureWait1:
	clrwdt
	movfw MonitorFrameCount
	bz MeasureWait1 ; First wait till MonitorFrameCount is nonzero (0xFF).
MeasureWait2:
	clrwdt
	movfw MonitorFrameCount
	bnz MeasureWait2 ; Then wait till its zero again (monitor frame complete).
	movfw VmonitorU
	movwf INDF
	decf FSR, f
	movfw VmonitorH
	movwf INDF
	return

; =============================================================================
Average:
	; Input: WREG points to the high-order byte of the first 16 bits value to average.
	; The second value should be immediate after the high-order byte of the first value.
	; Output: the first value is changed to the average of the two values.

	; Add the second value to the first:
	addlw 2
	movwf FSR
	movfw INDF
	decf FSR, f
	decf FSR, f
	addwf INDF, f
	incf FSR, f
	movfw INDF
	decf FSR, f
	decf FSR, f
	addwf INDF, f
	incf FSR, f
	addcf INDF, f
	
	; Divide the first value by two.
	clrc
	rrf INDF, f
	decf FSR, f
	rrf INDF, f
	return

; =============================================================================
WaitForKeyPress:
	clrf KeyState
KeyWait:	
	clrwdt
	btfss KeyState, KeyPressed
	goto KeyWait	
	return

; =============================================================================
	END