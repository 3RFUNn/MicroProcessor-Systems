	AREA myData, DATA, READONLY
		
COUNT	EQU	8
SUM		EQU	0

	AREA RESET, CODE, READONLY
	ENTRY
		LDR R0, =COUNT
		LDR	R1,	=SUM
		LDR	R2,	=1	; r2 stores the initial value of i
		
myloop
		ADD	R1, R2, R1	; sum = i + sum
		ADD	R2, R2, #1	; increment i
		SUBS R4, R0, R2	; r4 = r0 - r2 (check if r0 and r2 are equal)
						; if they are equal result is zero, z flag will set
		BNE	myloop		; if flag hasn't been set reapet the loop again
		ADD	R1, R2, R1	; make sure that we also add the final COUNT value
	
stop	B	stop

	END