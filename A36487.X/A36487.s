        .global _INT1Interrupt
	.text
_INT1Interrupt:	
	PUSH.S
	BSET 		LATA, #15 		; DPARKER - Debugging ONLY

	BTSS		PORTA, #12 		; Make sure the trigger pin is still active, it it is not, then it was noise on the trigger input
	BRA		_INT1DoNotTrigger
	BTSS		IFSO, #7 		; Check that _T3IF is set (this ensures that we have met minimum pulse period) 
	BRA		_INT1DoNotTrigger
	;; Check that we are in the X-Ray State
	MOV		global_data_A36487.control_state,  W0
	SUB		0x32, W0
	BRA		NZ, _INT1DoNotTrigger
	;; We are good to pulse
	BSET		LATD, #4 		; Start the Pulse
	
_INT1DoNotTrigger:	
	
	BLCR 		LATA, #15
	POP.S
	RETFIE
	
