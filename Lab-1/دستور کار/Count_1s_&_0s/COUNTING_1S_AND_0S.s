 AREA RESET, DATA, READONLY
  DCD          0    ;initial_sp
  DCD          MAIN;reset_vector
	  
 AREA SA,DATA,READWRITE
SS SPACE 0x18
 
 AREA MyCode,CODE,READONLY
      ENTRY
MAIN
		LDR R11, =SS
		ADD R11, R11,#0x00000018
		MOV SP, R11
	  
		MOV R0,	#0x1234  	; Register Rd.
		MOV R4,	#0x10    	; Register's length (counter (16 bit)).
START		
		MOVS R0, R0, LSR #1 ; Logical shift right of number.
		BCS	COUNTING_ONE	; Branch to COUNTING_ONE if crray flag was set . 
		BCC	COUNTING_ZERO	; Branch to COUNTING_zero if crray flag was NOT set. 
CONTINUE
		SUB R4, R4, #1      ; decrementing counter value.
		CMP R4, #0x0		; Check for end of the loop.
		BNE START			; If there was still any bits we run the loop again.
		BL	SUBR
		
		SUB R11, R11,#0x00000018
		MOV SP, R11
		
STOP	B	STOP			; End of the code.

COUNTING_ONE
		ADD	R1, R1, #1		; R3 holds number of 1s.
		B	CONTINUE		; Back to CONTINUE to decrement the counter and check the end of the loop.
		
		
COUNTING_ZERO
		ADD	R2, R2, #1		; R4 holds number of 0s.
		B	CONTINUE		; Back to CONTINUE to decrement the counter and check the end of the loop.
		
SUBR
	PUSH {R1, R2, LR}
	
	MOV	R5, #3
	MUL	R2, R1, R5		; R2 = R1 * 3
	MOV	R6, #100
	SUB	R3, R2, R6		; R3 = R2 - 100
	
	POP	{R1, R2, LR}
	
	BX	LR

      END