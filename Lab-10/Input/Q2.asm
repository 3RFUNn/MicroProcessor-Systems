.MODEL SMALL 
.STACK 100H 
.DATA    
    DATA1   DD 0
    DATA2   DD 0   
    DATA3   DD 0, 0
    DATA4   DD 0,
    TEN     DD 10
    R       DD 0 
    MUL_RES DW 0, 0, 0, 0
    SIGN    DW 0      

   

.CODE 
    MAIN PROC FAR 
        MOV AX,@DATA 
        MOV DS,AX        
         
        LEA BX, DATA1
        CALL GETNUM
        
        MOV DX,13
        MOV AH,2
        INT 21H  
        MOV DX,10
        MOV AH,2
        INT 21H
        
        LEA BX, DATA2
        CALL GETNUM 
        
        MOV DX,13
        MOV AH,2
        INT 21H  
        MOV DX,10
        MOV AH,2
        INT 21H 
        
        
        CALL MULTNUMS 
        CALL DIVNUMS 
        
        
        LEA BX, DATA3
        CALL PRINTNUM
        
        MOV DX,13
        MOV AH,2
        INT 21H  
        MOV DX,10
        MOV AH,2
        INT 21H
        
         
        LEA BX, DATA4
        CALL PRINTNUM        
        
        MOV AH,4CH 
        INT 21H 
    
    MAIN ENDP
    
    
    GETNUM PROC      

        
        MOV AH,1    
        INT 21H                                     
        CMP AL, 0X2D  
        JNE FIRST_CHAR
        XOR [SIGN], 1
                
GET_CHAR:   
        MOV AH,1    
        INT 21H     
FIRST_CHAR:                   
        MOV CL,AL 
        MOV CH, 0  
        SUB CX, 48             
        CMP AL, 13
        JE CHECK_SIGN       
        MOV AX,[BX]         
        MUL [TEN]      
        MOV [MUL_RES],AX
        MOV [MUL_RES + 2],DX       
        MOV AX,[BX + 2]       
        MUL [TEN]      
        ADD [MUL_RES + 2],AX
        ADC [MUL_RES + 4],DX       
        MOV AX,[BX]        
        MUL [TEN + 2]     
        ADD [MUL_RES + 2],AX
        ADC [MUL_RES + 4],DX       
        ADC [MUL_RES + 6], 0   
        MOV AX,[BX + 2]        
        MUL [TEN + 2]     
        ADD [MUL_RES + 4],AX
        ADC [MUL_RES + 6],DX 
        
        ADD [MUL_RES], CX
        ADC [MUL_RES + 2], 0
        ADC [MUL_RES + 4], 0       
        ADC [MUL_RES + 6], 0
        
        MOV AX, [MUL_RES]
        MOV [BX], AX 
        MOV AX, [MUL_RES+2]
        MOV [BX+2], AX 
           
        JMP GET_CHAR    

CHECK_SIGN:        
        CMP [SIGN], 0
        JE RES_POS      
        NOT [BX]  
        NOT [BX + 1]     
        NOT [BX + 2]
        NOT [BX + 3] 
        
        ADD [BX], 1
        ADC [BX + 1], 0
        ADC [BX + 2], 0
        ADC [BX + 3], 0
RES_POS:         

        MOV [SIGN], 0        
        RET
    GETNUM ENDP
    
    
    PRINTNUM PROC 
        MOV DH, 0
        MOV DL, '!'     
        PUSH DX 
        
        MOV AX, [BX + 2]
        AND AX, 0X8000    
        CMP AX, 0
        JE  PRINT_RES_POS
        MOV DL, '-'
        MOV AH,2              
        INT 21H 
        NOT [BX]  
        NOT [BX + 1]     
        NOT [BX + 2]
        NOT [BX + 3] 
        
        ADD [BX], 1
        ADC [BX + 1], 0
        ADC [BX + 2], 0
        ADC [BX + 3], 0
               
PRINT_RES_POS:  
        CMP [BX + 2], 0
        JNE NOT_ZERO   
        CMP [BX], 0
        JNE NOT_ZERO
        JMP DISPLAY
NOT_ZERO:             
        XOR DX,DX              
        MOV AX, [BX+2]    
        DIV [TEN]                    
        PUSH AX
        MOV AX, [BX]      
        DIV [TEN]                   
        MOV [BX], AX   
        POP [BX + 2]
        PUSH DX  
        JMP PRINT_RES_POS
DISPLAY:
        POP DX
        CMP DL, '!'                
        JE DISP_DONE
        ADD DL, 48 
        MOV AH,2              
        INT 21H 
        JMP DISPLAY
DISP_DONE:
        MOV [SIGN], 0                
        RET
    PRINTNUM ENDP 
    
    
    
    MULTNUMS PROC  
        
        PUSH [DATA1]
        PUSH [DATA1 + 2]   
        PUSH [DATA2]
        PUSH [DATA2 + 2]
        
        MOV AX, 0X8000
        AND AX, [DATA1 + 2] 
        CMP AX, 0
        JE DATA1_POS    
        XOR [SIGN], 1
   
        NOT [DATA1]
        NOT [DATA1 + 2]
        ADD [DATA1], 1 
        ADC [DATA1 + 2], 0
              
DATA1_POS:            
        MOV AX, 0X8000
        AND AX, [DATA2 + 2] 
        CMP AX, 0
        JE DATA2_POS    
        XOR [SIGN], 1 
        
        NOT [DATA2]
        NOT [DATA2 + 2]
        ADD [DATA2], 1 
        ADC [DATA2 + 2], 0
               
DATA2_POS:             
        MOV AX,[DATA1]         
        MUL [DATA2]      
        MOV [DATA3],AX
        MOV [DATA3 + 2],DX       
        MOV AX,[DATA1 + 2]       
        MUL [DATA2]      
        ADD [DATA3 + 2],AX
        ADC [DATA3 + 4],DX       
        MOV AX,[DATA1]        
        MUL [DATA2 + 2]     
        ADD [DATA3 + 2],AX
        ADC [DATA3 + 4],DX       
        ADC [DATA3 + 6], 0   
        MOV AX,[DATA1 + 2]        
        MUL [DATA2 + 2]     
        ADD [DATA3 + 4],AX
        ADC [DATA3 + 6],DX      
         
        
        CMP [SIGN], 0
        JE MUL_RES_POS       
        NOT [DATA3]     
        NOT [DATA3 + 2]
        NOT [DATA3 + 4]       
        NOT [DATA3 + 6]
        
        ADD [DATA3], 1
        ADC [DATA3 + 2], 0
        ADC [DATA3 + 4], 0
        ADC [DATA3 + 6], 0
MUL_RES_POS:
        
        MOV [SIGN], 0
        POP [DATA2 + 2]
        POP [DATA2]   
        POP [DATA1 + 2]
        POP [DATA1]
                
        RET
    MULTNUMS ENDP  
    
    
    
    
    DIVNUMS PROC NEAR
        
         
        PUSH [DATA1]
        PUSH [DATA1 + 2]   
        PUSH [DATA2]
        PUSH [DATA2 + 2] 
        
        MOV AX, 0X8000
        AND AX, [DATA1 + 2] 
        CMP AX, 0
        JE DATA1_POS_DIV    
        XOR [SIGN], 1
   
        NOT [DATA1]
        NOT [DATA1 + 2]
        ADD [DATA1], 1 
        ADC [DATA1 + 2], 0
              
DATA1_POS_DIV:            
        MOV AX, 0X8000
        AND AX, [DATA2 + 2] 
        CMP AX, 0
        JE DATA2_POS_DIV    
        XOR [SIGN], 1 
        
        NOT [DATA2]
        NOT [DATA2 + 2]
        ADD [DATA2], 1 
        ADC [DATA2 + 2], 0
               
DATA2_POS_DIV:       
        
        
        MOV CL, 32
        
DIV_LOOP:
        DEC CL
         
         
        SHL [R + 2], 1
        MOV AX, [R]
        SHR AX, 15
        OR  [R + 2], AX  
        SHL [R], 1
        
        
          
        CMP CL, 15  
        JA ABOVE1
        MOV AX, [DATA1]
        SHR AX, CL
        AND AX, 1
        OR  [R], AX 
        
        ABOVE1:
        MOV AX, [DATA1 + 2] 
        SUB CL, 16
        SHR AX, CL
        ADD CL, 16
        AND AX, 1
        OR  [R], AX 
        JMP CALC
   
        
CALC:        
        MOV AX, [DATA2 + 2]
        CMP AX, [R + 2]
        JA CHECK_LOOP
        JB BELOW 
        MOV AX, [DATA2]
        CMP AX, [R]
        JA CHECK_LOOP        
BELOW: 
        MOV AX, [DATA2 + 2]   
        MOV DX, [DATA2]
        SUB [R], DX
        SBB [R + 2], AX
        
        CMP CL, 15
        JA ABOVE
        MOV AX, 1
        SHL AX, CL
        OR [DATA4], AX
        JMP CHECK_LOOP
        
ABOVE:
        MOV AX, 1
        SUB CL, 16
        SHL AX, CL
        ADD CL, 16
        OR [DATA4 + 2], AX
        
                                        
                       
CHECK_LOOP:         
        CMP CL, 0
        JNE DIV_LOOP
        CMP [SIGN], 0
        JE DIV_RES_POS       
        NOT [DATA4]     
        NOT [DATA4 + 2]
        NOT [DATA4 + 4]       
        NOT [DATA4 + 6]
        
        ADD [DATA4], 1
        ADC [DATA4 + 2], 0
        ADC [DATA4 + 4], 0
        ADC [DATA4 + 6], 0
DIV_RES_POS:
        
        MOV [SIGN], 0
        POP [DATA2 + 2]
        POP [DATA2]   
        POP [DATA1 + 2]
        POP [DATA1]
        RET
    DIVNUMS ENDP

END MAIN