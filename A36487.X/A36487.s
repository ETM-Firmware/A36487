        .global _INT1Interrupt
	.text
_INT1Interrupt:	
	PUSH.S
	BSET 		LATA, #15

	BTSS		PORTA, #12 		; Make sure the trigger pin is still active, it it is not, then it was noise on the trigger input
	BRA		_INT1DoNotTrigger
	BTSS		IFSO, #7 		; Check that _T3IF is set (this ensures that we have met minimum pulse period) 
	BRA		_INT1DoNotTrigger
	;; Check that we are in the X-Ray State
	MOV		50, W0
	SUB		global_data_A36487.control_state,  W0
	
	;; We are good to pulse

	
_INT1DoNotTrigger:	
	
	BLCR 		LATA, #15
	POP.S
	RETFIE
	
