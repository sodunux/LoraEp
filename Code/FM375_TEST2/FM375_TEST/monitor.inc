;****************************
; BIRD_CTRLSTS definition
;****************************
BIRD_CTRLSTS    DATA   08EH

  CLRSOFTBKPSTS  EQU 080h
  CR_DATA       EQU 020h
  XCH_DATA      EQU 010h

  BIT_SOFTBKPSTS     EQU 004h
  BIT_INTRMODE      EQU 002h
  BIT_RDY2WR        EQU 001h
  BIT_CMD_AVAILABLE  EQU 000h

;****************************
; DATA_REG definition
;****************************
DATA_REG   DATA   08Fh


;****************************
;Command definition
;****************************
CMD_READ         EQU 000h
CMD_WRITE        EQU 008h
BIT_WRITE        EQU 003h

CMD_GO           EQU 040h
BIT_GO_FCT       EQU 006h


;****************************
;Space defintion
;****************************
SPACE_SFR      EQU 000h
SPACE_DATA     EQU 001h
SPACE_REG      EQU 002h
