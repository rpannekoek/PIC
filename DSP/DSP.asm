	#include p18F2420.inc
	
	radix dec
	
	; RAM variables
	CBLOCK 0
		OutputRoutinePCL
		WaveformBaseH
		WaveformStepL
		WaveformStepH
	ENDC
	
; =========================================================================
	ORG 0
ResetVector:
	; TODO
	movlw WaveformOutputLF
	movwf OutputRoutinePCL	
	goto HighPrioIntVector
	
; =========================================================================
	ORG 8
HighPrioIntVector:
	movf OutputRoutinePCL, ACCESS
	movwf PCL, ACCESS
	
; =========================================================================
	ORG 0x18
LowPrioIntVector:
	; TODO: save and restor WREG, STATUS and BST (fast save/restore is used for High Prio Interrupts)
	retfie 0;
	

; =========================================================================
WaveformOutputLF:
	; Low-Frequency waveform output. Performed with 204.8 kHz sample frequency.
	; With step size of 1 (2048 samples per cycle), min. frequency is 100 Hz.
	; For lower frequencies, sample frequency can be reduced further (aliasing is very low anyways; ZOH is sufficient filter.)
	; For higher frequencies, step size is increased, giving 100 Hz increments.
	; To prevent aliasing (LPF cut-off frequency is 100 kHz), step size should be kept low.
	; Total cycles for interrupt handling (incl. interrupt latency): 19 => max. sample freq = 500 kHz (@ 40 MHz clock)
	
	; Fetch waveform byte from program memory and output to DAC latch
	tblrd*
	movff TABLAT, PORTB

	; Advance waveform table pointer
	movf WaveformStepL, w
	addwf TBLPTRL, f
	movf WaveformStepH, w
	addwfc TBLPTRH, w
	andlw 0x7 ; wrap after 2 kB (11 bits)
	iorwf WaveformBaseH
	movwf TBLPTRH

	retfie 1 ; Restore WREG, STATUS and BSR from their shadow registers	

; =========================================================================
WaveformOutputHF:
	; High-Frequency waveform output. Performed with 512 kHz sample frequency.
	; Using a fixed 256 byte RAM buffer, which is prefilled with waveform samples at the desired frequency.
	; Step size is always 1, resulting in 2 kHz base frequency (256 sample cycle).
	; The buffer must contain an integer number of cycles, giving 2 kHz increments.
	; With 4 sample cycle, max. output frequency is 128 kHz.
	; Total cycles for interrupt handling (incl. interrupt latency): 12 => max. sample freq = 800 kHz (@ 40 MHz clock)
	
	; Fetch waveform byte from RAM and output to DAC latch.
	movff INDF0, PORTB
	
	; Advance waveform table pointer.
	incf FSR0L  ; wrap after 256 bytes.
	
	retfie 1 ; Restore WREG, STATUS and BSR from their shadow registers	
	
	end
	