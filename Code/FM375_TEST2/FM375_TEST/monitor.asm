$include (reg251s.inc)
$include (monitor.inc)
$include (startaddr.inc)

;***************************************************
; monitor program
;***************************************************

IF STARTADDR <> 0
CSEG AT 07Bh
  EJMP  readcontext
ENDIF
                      
CSEG AT STARTADDR

MAIN:
    ; configure BIRD in communication mode
    mov BIRD_CTRLSTS, #XCH_DATA
;***************************************************
; Wait command loop
; ==========================
; This loop is executed in monitor to wait order from BIRD plug-in
; It read BIRD_CTRLSTS to check if command is available and read
; it if needed
;***************************************************
WAIT_CMD:
    ; wait that a command is available and put command in A
    lcall WAIT_BYTE_MACRO_A  
    ;Test if command is GO command
    jnb ACC.BIT_GO_FCT, CMD_ANALYSIS
    ljmp GO_FCT

CMD_ANALYSIS:
    ;;;;;;;;;;;;;;;;;;
    ; save command in R0 and in R1
    mov R0, ACC
    mov R2, ACC    
    ;mov B, ACC
    ; keep only spacce in R0
    anl R0, #007h
    ;;;;;;;;;;;;;;;;;;
    ; Read bytes number to read/write
    
    ; get counth
    lcall WAIT_BYTE_MACRO
    mov R3, DATA_REG
    ; get countl
    lcall WAIT_BYTE_MACRO
    ; countH is not used so it is overwritten by countl
    mov R3, DATA_REG
    
    ;;;;;;;;;;;;;;;;;;
    ; Read Start Address to read write
    lcall WAIT_BYTE_MACRO
    mov DPXL, DATA_REG ; addrH
    lcall WAIT_BYTE_MACRO
    mov DPH, DATA_REG ; addrM
    lcall WAIT_BYTE_MACRO
    mov DPL, DATA_REG ; addrL

    ;;;;;;;;;;;;;;;;;;
    ;Code replace configuration
    ; write code replace reg mode
    mov BIRD_CTRLSTS, #CR_DATA
    ; write code replace register
    mov DATA_REG, DPL
    
    ; communication mode
    mov BIRD_CTRLSTS, #XCH_DATA
    
    ljmp test_space
    
END_COMMAND:
    dec R3
    inc dptr
    ;;;;;;;;;;;;;;;;;;
    ;Code replace configuration
    ; write code replace reg mode
    mov BIRD_CTRLSTS, #CR_DATA
    ; write code replace register
    mov DATA_REG, DPL
    
    ; communication mode
    mov BIRD_CTRLSTS, #XCH_DATA

    ; restart read/write procedure if it left bytes to read/write
    cjne R3, #000h, TEST_SPACE
    ljmp WAIT_CMD

TEST_SPACE:
    ; rewrite command in B
    mov B, R2
TEST_SFR:
    cjne R0, #SPACE_SFR, test_data
    ljmp sfr_space

TEST_DATA:
    cjne R0, #SPACE_DATA, REG_SPACE_i
    ljmp DATA_SPACE

REG_SPACE_i:
    ljmp REG_SPACE

;***************************************************
; TRAP interrupt handler
; ==========================
; This function is the interrupt handler of the TRAP
; interrupt. It saves the minimum context to run this 
; monitor programm to avoid to break any customer values.
;***************************************************

cseg at STARTADDR+07Bh

readcontext:
    ;;;;;;;;;;;;;;;;;;
    ; configure in communication mode
    mov BIRD_CTRLSTS, #XCH_DATA

    ; save B                             B
    mov DATA_REG, B                     
    lcall WAIT_END_OF_WRITE_MACRO

    ;save A                              A 
    mov DATA_REG, ACC
    lcall WAIT_END_OF_WRITE_MACRO

//#ifdef MONI_REAL
    ;save PSW                            PSW
    mov DATA_REG, PSW
    lcall WAIT_END_OF_WRITE_MACRO
    ;use bank 0 in the monitor
    anl PSW, #0E7h  
    
    ; save DPL                           DPL
    mov DATA_REG, DPL                       
    lcall WAIT_END_OF_WRITE_MACRO

    ;save DPH                            DPH
    mov DATA_REG, DPH
    lcall WAIT_END_OF_WRITE_MACRO

    ;save DPXL                           DPXL
    mov DATA_REG, DPXL
    lcall WAIT_END_OF_WRITE_MACRO
//#endif MONI_REAL

    ; get information on bkp soft and irq mode
    mov ACC, BIRD_CTRLSTS
    
    ; clear information on soft bkp
    mov BIRD_CTRLSTS, #XCH_DATA ; => CLRSOFTBKPSTS = 0

    ; send status information             info
    ;mov CCM_VAL, ACC
    ;lcall WAIT_END_OF_WRITE_MACRO

                                   
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;Read PC and send it
    ;;;;;;;;;;;;;;;;;;
    jnb ACC.BIT_INTRMODE, TWO_BYTES_STACK_SAVE
    ; get stack in B                      PCM (intr4)
    pop B
    mov DATA_REG, B
    lcall WAIT_END_OF_WRITE_MACRO
    ; get stack in B                      PCL (intr4)
    pop B
    mov DATA_REG, B
    lcall WAIT_END_OF_WRITE_MACRO

TWO_BYTES_STACK_SAVE:
    ; get stack in B                      PCM (intr2) or PCH (intr4)

    pop B
    mov DATA_REG, B                         
    lcall WAIT_END_OF_WRITE_MACRO
    ; get stack in B                      PCL (intr2) or PSW1 (intr4)
    pop B                
    mov DATA_REG, B
    lcall WAIT_END_OF_WRITE_MACRO
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ; Send 4 register banks through memory access

//#ifdef MONI_REAL
    ; Bank 0 is used in monitor
	;add by lijinhong
	anl PSW,#0E7h
    ;save R0 first                        @00 0000
    mov DATA_REG, R0
    lcall WAIT_END_OF_WRITE_MACRO	

    ;save the registers
    mov R0, #01h
    ;                                     @00 0001 -> @00 001F			  
save_register:
    mov DATA_REG, @R0
    lcall WAIT_END_OF_WRITE_MACRO
    inc R0
    cjne  R0, #20h, save_register
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
//#endif MONI_REAL

    ljmp main
    

;***************************************************
; GO function
; ======================
; This function restore the saved context before 
; jumping into the user application.
;***************************************************
;org 0CAh
GO_FCT:

//#ifdef MONI_REAL
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    ; Restore 4 register banks through memory access
    
    ;save the registers
    mov R0, #01Fh
    ;                                     @00 001F -> @00 0001
restore_register:
    lcall WAIT_BYTE_MACRO        
    mov @R0, DATA_REG
    dec R0 
    cjne  R0, #00h, restore_register
    ;restore R0                          @00 0000
    lcall WAIT_BYTE_MACRO        
    mov R0, DATA_REG
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
//#endif MONI_REAL


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    ; read information on status to know if 2 or
    ; for bytes have to be write in stack   
    mov ACC, BIRD_CTRLSTS

    mov BIRD_CTRLSTS, #XCH_DATA

    jnb ACC.BIT_INTRMODE, TWO_BYTES_STACK_RESTORE
    ; restore PC                          PSW1
    lcall WAIT_BYTE_MACRO_A
    push ACC
    ;                                     PCH
                                   
    lcall WAIT_BYTE_MACRO_A        
    push ACC
TWO_BYTES_STACK_RESTORE:
    ;                                     PCL
    lcall WAIT_BYTE_MACRO_A
    push ACC
    ;                                     PCM
    lcall WAIT_BYTE_MACRO_A
    push ACC
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    

   
    ; this byte is not used but sent by plugin
    ;                                    info
    ;lcall WAIT_BYTE_MACRO_A                   

//#ifdef MONI_REAL

    ; restore DPXL                       DPXL
    lcall WAIT_BYTE_MACRO
    mov DPXL, DATA_REG
    ; restore DPH                        DPH
    lcall WAIT_BYTE_MACRO
    mov DPH, DATA_REG
    ; restore DPL                        DPL
    lcall WAIT_BYTE_MACRO
    mov DPL, DATA_REG

    ; restore PSW                        PSW
    lcall WAIT_BYTE_MACRO
    mov PSW, DATA_REG
//#endif MONI_REAL

    ; restore A                          A
    lcall WAIT_BYTE_MACRO
    mov ACC, DATA_REG   
    ; restore B                          B
    lcall WAIT_BYTE_MACRO
    mov B, DATA_REG    

    reti


SFR_SPACE:
    ;test read or write
    jb B.BIT_WRITE, SFR_WRITE
SFR_READ:
		;mov R0,DPL
		;mov A,@R0
    mov R7, 0B1h 			;mov R7, 080h      80  has to replace by code replace
    mov DATA_REG, R7 	;mov DATA_REG, R7
    lcall WAIT_END_OF_WRITE_MACRO
    ljmp END_COMMAND

SFR_WRITE:
    lcall WAIT_BYTE_MACRO
    mov 0B1h, DATA_REG ;mov 080h, DATA_REG   80  has to replace by code replace
    ljmp END_COMMAND
    
    
DATA_SPACE:
    ;test read or write
    jb B.BIT_WRITE, DATA_WRITE
DATA_READ:
    movx A, @dptr
    mov DATA_REG, A
    lcall WAIT_END_OF_WRITE_MACRO
    ljmp END_COMMAND
    
DATA_WRITE:
    lcall WAIT_BYTE_MACRO
    mov A, DATA_REG
    movx @dptr, A
    ljmp END_COMMAND
    
REG_SPACE:
    ;test read or write
    jb B.BIT_WRITE, REG_WRITE
REG_READ:
    mov DR4, DR16 ; DR4, DR16 will be replace by code replace

    mov R1, #004h
SEND_DR4:
    mov DATA_REG, @R1
    inc R1
    lcall WAIT_END_OF_WRITE_MACRO
    cjne R1, #008h, SEND_DR4
    ljmp END_COMMAND

REG_WRITE:

    mov R1, #004h
WRITE_DR4:
    lcall WAIT_BYTE_MACRO
    mov @R1, DATA_REG
    inc R1
    cjne R1, #008h, WRITE_DR4
    mov DR16, DR4 ; DR16,DR4 has to be replace by code replace
    ; it is only possible to write in DRk one byte by one byte
    
    ljmp WAIT_CMD

;***************************************************
; WAIT_BYTE function
; ==========================
; Wait a byte send by the BIRD module
;***************************************************
WAIT_BYTE_MACRO:
  mov B, BIRD_CTRLSTS
  jnb B.BIT_CMD_AVAILABLE, WAIT_BYTE_MACRO
  ret

;***************************************************
; WAIT_BYTE function A
; ==========================
; Wait a byte send by the BIRD module and placed it in ACC resgister if there is one
;***************************************************
WAIT_BYTE_MACRO_A:                                
  mov A, BIRD_CTRLSTS
  jnb ACC.BIT_CMD_AVAILABLE, WAIT_BYTE_MACRO_A
  mov A, DATA_REG
  ret
    
;***************************************************
; WAIT_END_OF_WRITE function
; ==========================
; Check if the CPU can send value to the BIRD
;***************************************************    
WAIT_END_OF_WRITE_MACRO:
  mov B, BIRD_CTRLSTS
  jnb B.BIT_RDY2WR, WAIT_END_OF_WRITE_MACRO
  ret
      
end
