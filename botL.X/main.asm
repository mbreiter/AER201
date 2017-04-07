;*******************************************************************************
;
;    Filename: main.asm
;    Date: 2 Feb 2017
;    File Version: 1.0
;    Author: Matthew Reiter
;    Course: AER201
;    Description: botL - a pastic bottle sorting machine
 
;*******************************************************************************
; configuration settings
;*******************************************************************************

list    P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC
#include <p18f4620.inc>
#include <lcd.inc>
#include <rtc.inc>
#include <colour.inc>
#include <sorting.inc>

    CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
    CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
    CONFIG WDT = OFF, WDTPS = 32768
    CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
    CONFIG STVREN = ON, LVP = OFF, XINST = OFF
    CONFIG DEBUG = OFF
    CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
    CONFIG CPB = OFF, CPD = OFF
    CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
    CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
    CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
    CONFIG EBTRB = OFF

;*******************************************************************************
; variable and constants
;*******************************************************************************
    CODE
	    
    cblock 0x00
	key1
	key2
	key3
	keyA
	key4
	key5
	key6
	keyB
	key7
	key8
	key9
	keyC
	keyS
	key0
	keyH
	keyD
	KEY
	temp_KEY
	KEY_ISR
	temp_S	
	temp_W
	counter
	counter2
	temp
	OP_sec
	OP_INT
	delayR
	d5us
	d200us
	d50ms
	inExecution
	clear_EE
	H_EE	
	L_EE	
	REG_EE  
	tempH_EE
	tempL_EE
	tempL_EEC
	MAX_EE   
	last_log
	transferring
	PC_PCL
	PC_PCLATH
	PC_PCLATU
	DETECTION_VAL
	ESKA
	ESKA_NOCAP
	YOP
	YOP_NOCAP
	TOTAL_BOTTLES
	COLLECTIONS_COUNT
	TRAY_COUNT
	TRAY_DELAY
	TRAY_CURRENT
	TRAY_GOTO
	STOP_CONDITION
	inStandby
    endc
    
    extern tens_digit, ones_digit, databyte
    extern WRITE_ARDUINO, READ_ARDUINO, INIT_RTC, INIT_ARDUINO
    
;*******************************************************************************
; tables
;*******************************************************************************
    Welcome	db	    "botL", 0
    Team	db	    "mr hl hg", 0
    StandBy	db	    "standing by ... ... standing by ... ...", 0
    StandbyInfo db	    "<a>sort <b>last log <c>perm logs <d>pc", 0
    Log1	db	    "time:",0
    Log2	db	    "12:00 2/3/14", 0
    LogInfo1	db	    "saved:", 0
    LogInfo2	db	    "back <0>", 0
    PermLog1	db	    "permanent logs ", 0
    PermLog2	db	    "access up to 9 logs, press <1> ... <9>", 0	
    Exe1	db	    "sorting...", 0
    Exe2	db	    "end <*>", 0
    PC1		db	    "pc interface ... pc interface ... ", 0
    PC2		db	    "press <#> to begin log transfer", 0
    PCTransfer	db	    "transferring...", 0
    PCComplete	db	    "complete!", 0
    SAVE	db	    "saving...", 0
    NoData	db	    "n/a", 0
	
;*******************************************************************************
; macros
;*******************************************************************************
	
ConfigLCD   macro
          movlw     b'00101000'    ; 4 bits, 2 lines,5X7 dots seems to work best instead of the above setting
          call      WR_INS

          movlw     b'00001100'    ; display on/off
          call      WR_INS
          movlw     b'00000001'    ; Clear ram
          call      WR_INS
	  endm

Delay50N macro count, N
	local	loop
	movlw	N
	movwf	count

loop
	call	Delay50ms
	decfsz	count
	goto	loop
	endm

Display	macro   Message
	local   loop
	movlw   upper Message	; Move Table<20:16>
	movwf   TBLPTRU
	movlw   high Message	; Move Table<15:8>
	movwf   TBLPTRH
	movlw   low Message	; Move Table<7:0>
	movwf   TBLPTRL
	tblrd*			; Read byte from TBLPTRL to TABLAT
	movf    TABLAT, W
loop
	call    WR_DATA		; write to LCD
	tblrd+*			; next row in table
	movf    TABLAT, W
	bnz	loop
	endm	

incf_BCD    macro   BCD
	local ones, endBCD
	movff	BCD, temp
	
	;process ones digit
	movlw	0x0f
	andwf	temp
	movlw	d'9'
	
	;skip if ones digit is 9
	cpfseq	temp
	goto	ones
	
	movlw	b'00010000'
	addwf	BCD
	movlw	0xf0
	andwf	BCD
	movlw	0xa0
	cpfslt	BCD
	
	clrf	BCD
	goto	endBCD
	
ones
	incf	BCD
	goto	endBCD
endBCD
	nop
	endm

WriteEE	macro   word, addH, addL
	movff   addH, EEADRH	    ; High address
	movff   addL, EEADR	    ; Low address
	movff   word, EEDATA	    ; set word

	btfsc   EECON1, WR	    ; test if WR=0, skip if so
	bra	$-2

	bcf	EECON1, EEPGD	    ; Point to data memory
	bcf	EECON1, CFGS	    ; Access EEPROM
	bsf	EECON1, WREN	    ; enable write
	bcf	INTCON, GIE	    ; disable interrupts during write
	bcf	PIR2, EEIF

	movlw   0x55
	movwf   EECON2		    ; write 55h
	movlw   0xaa
	movwf   EECON2		    ; write 0xaa
	bsf	EECON1, WR	    ; begin write
	btfsc   EECON1, WR
	bra	$-2

	bsf	INTCON, GIE	    ; enable interrupts after write
	bcf	EECON1, WREN	    ; disable writes
	endm

READEE	macro   file, addH, addL
	movff   addH, EEADRH	    ; read to high
	movff   addL, EEADR	    ; read to low
	bcf	EECON1, EEPGD	    ; point to data memory
	bcf	EECON1, CFGS	    ; access to EEPROM
	bsf	EECON1, RD	    ; read EEPROM
	movff   EEDATA, file	    ; put data into file
	endm

ChangeMode  macro   ModeId, NextMode
	local Next, Exit

	movlw	ModeId		    ; poll for mode
	cpfseq	KEY		    ; check this against key press
	goto	Exit

Next
	clrf	LATA		    ; clear pins before proceeding
	clrf	LATB
	clrf	LATC
	clrf	LATD
	goto	NextMode

Exit
	nop
	endm

saveContext macro
    movff   STATUS, temp_S	    ; save STATUS first
    movwf   temp_W                  ; save W
    endm

restoreContext macro
    swapf   temp_W, 0               ; restore W first
    movff   temp_S, STATUS	    ; restore STATUS last without affecting W
    endm
;*******************************************************************************
; reset vector and isrs
;*******************************************************************************
	ORG	0x000		    ; processor reset vector
	goto    INIT		    ; go to beginning of program

	ORG	0x008
	goto    ISR_HIGH

	ORG	0x018
	goto    ISR_LOW

LOAD_STANDBY_TIME
	btfss	T0CON, TMR0ON
	return
	
	movlw	0xff
	movwf	TMR0H
	movlw	0xff
	movwf	TMR0L

	return	

LOAD_EXE_TIME
	btfss	T0CON, TMR0ON
	return
	
	movlw	0xc3
	movwf	TMR0H
	movlw	0x28
	movwf	TMR0L
	
	return
	
ISR_HIGH
	saveContext
	
	movlw	0x00
	cpfseq	inStandby
	call	Shift
	
	;reset timer, but need to check which time increment 
	movlw	0x00
	cpfseq	inStandby
	call	LOAD_STANDBY_TIME
	
	movlw	0xff
	cpfseq	inStandby
	call	LOAD_EXE_TIME
		
	;timer interrupt
	btfss	INTCON, TMR0IF
	goto	END_ISR_HIGH
	
	incf_BCD    OP_INT
	movlw	d'0'
	cpfseq	OP_INT
	goto	END_ISR_HIGH
	incf_BCD    OP_sec

END_ISR_HIGH
	bcf	INTCON, TMR0IF
	restoreContext
	retfie

ISR_LOW
	saveContext
	; KEYPAD INTERRUPT
	btfss	INTCON3, INT1IF		; If KEYPAD interrupt, skip return
	goto	END_ISR_LOW

	; check operation status
	movlw	0xff			; If in operation, skip return
	cpfseq	inExecution
	goto	END_ISR_LOW

	; process KEY
	swapf	PORTB, 0		; Read PORTB<7:4> into W<3:0>
	andlw	0x0f
	movwf	KEY_ISR
	
	movlw	keyS			; Put keyStar into W to compare to KEY_ISR
	cpfseq	KEY_ISR			; If key was '*', skip return
	goto	END_ISR_LOW
	
	; reset program counter - emergency stop recorded. 
	bcf	T0CON, TMR0ON
	call	SAVE_EXE_TIME
		
	; Clear inExecution flag to stop '*' interrupts
	clrf	inExecution
	movlw	d'1'
	movwf	STOP_CONDITION

	call	SaveData
	
	clrf	TOSU
	clrf	TOSH
	clrf	TOSL
	bcf	INTCON3, INT1IF         ; Clear flag for next interrupt
	restoreContext
	retfie

END_ISR_LOW
	bcf			INTCON3, INT1IF         ; Clear flag for next interrupt
	restoreContext
	retfie

;*******************************************************************************
; main
;*******************************************************************************
INIT
	movlw	b'01110000'	;Set internal oscillator frequency to 8MHz
	movwf	OSCCON
	
	; i/o
	movlw	b'00000000'
	movwf	TRISA
	movlw	b'11111111'
	movwf	TRISB		    ; keypad
	movlw	b'00000000'
	movwf	TRISC
	movlw	b'00000000'
	movwf	TRISD
	movlw	b'00000011'
	movwf	TRISE

	; clear ports
	movlw	b'00000000'
	movwf	LATA
	movlw	b'00000000'
	movwf	LATB
	movlw	b'00000000'
	movwf	LATC
	movlw	b'00000000'
	movwf	LATD
	movlw	b'00000000'
	movwf	LATE
	
	movlw	b'00000000'
	movwf	ADCON0
	
	; initializations
	call	InitLCD
	ConfigLCD
	
	call	i2c_common_setup
	call	RTC_INIT
	call	INIT_ARDUINO

	;COLOUR_INIT
	call	INIT_USART

	; interrupts
	clrf	RCON
	clrf	INTCON
	clrf	INTCON2
	clrf	INTCON3
	bsf	RCON, IPEN	    ; priority mode
	bsf	INTCON, GIEH	    ; permit global interrupts
	bsf	INTCON, GIEL
	bsf	INTCON2, INTEDG1    ; INTEDG1 on rising edge
	bsf	INTCON, TMR0IE	    ; clear timer0 overflow interrupt flag
	bsf	INTCON, TMR0IF	    ; clear timer0 overflow interrupt flag
	bsf	INTCON2, TMR0IP	    ; set to high priority
	bsf	INTCON3, INT1IE	    ; enable interrupts on rb1 for keyboard
	bcf	INTCON3, INT1IP	    ; keyboard to low priority
	
	; setting up timer to sychronize with 1 second clock intervals
	bcf	T0CON, TMR0ON
	bcf	T0CON, T08BIT
	bcf	T0CON, T0CS
	bcf	T0CON, T0SE
	bcf	T0CON, PSA
	bcf	T0CON, T0PS2	    ; set prescaling to 1:16.
	bsf	T0CON, T0PS1
	bsf	T0CON, T0PS0

	clrf	H_EE
	clrf	L_EE
	clrf	tens_digit
	clrf	ones_digit
	
	;ensure that the motor is indeed off
	Delay50N delayR, 0x03
	movlw	d'0'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	
	clrf	DETECTION_VAL
	clrf	ESKA
	clrf	ESKA_NOCAP
	clrf	YOP
	clrf	YOP_NOCAP
	clrf	TOTAL_BOTTLES
	clrf	COLLECTIONS_COUNT
	
	movlw     b'11110010'    ; Set required keypad inputs
        movwf     TRISB
	movlw	b'00000011'
	movwf	TRISE
	
	call	ClrLCD
	Display	Welcome
	call LCD_L2
	Display	Team
	
	Delay50N delayR, 0x28

;*******************************************************************************
; standby mode
;*******************************************************************************
STANDBY
	movlw	b'00000000'
	movwf	PORTE
	movlw	b'00000000'
	movwf	PORTA
	
	
	setf	inStandby	
	call	ClrLCD
	Display	StandBy
	call	LCD_L2
	Display	StandbyInfo
	
	movlw	0xff
	movwf	TMR0H
	movlw	0xff
	movwf	TMR0L
	bsf	T0CON, TMR0ON	    ; turning on timer

HOLD_STANDBY
	call	READ_KEY_TIME

	ChangeMode  key1, COLOUR_TEST
	ChangeMode  keyA, EXECUTION
	ChangeMode  keyB, LOG
	ChangeMode  keyC, PERM_LOG
	ChangeMode  keyD, PC_MODE
	bra	HOLD_STANDBY
	
COLOUR_TEST
	call	ClrLCD
	clrf	inStandby
	movlw	b'00000001'
	movwf	PORTE

LOOPING
	Delay50N delayR, 0x3c
	call	READ_ARDUINO
	movwf	DETECTION_VAL
	addlw	0x30
	call	WR_DATA
	Delay50N delayR, 0x3c
	
	bra COLOUR_TEST

;*******************************************************************************
; execution mode
;*******************************************************************************
	
EXECUTION
	;bcf	T0CON, TMR0ON	    ; turning off standby timer
	clrf	inStandby	
	call	ClearEEPROM_21
	
	; save the starting time
	call	SAVE_TIME
		
	; display
        setf	inExecution
	call	ClrLCD
	Display	Exe1
	
	movlw	0xc3
	movwf	TMR0H
	movlw	0x28
	movwf	TMR0L
	
	bsf	T0CON, TMR0ON	    ; turning on timer
	
	; initialize variables
	clrf	OP_sec
	clrf	OP_INT
	
	clrf	ESKA
	clrf	ESKA_NOCAP
	clrf	YOP
	clrf	YOP_NOCAP
	clrf	TOTAL_BOTTLES
	movlw	d'1'
	movwf	TRAY_CURRENT
	clrf	TRAY_DELAY
	
	movlw	d'1'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	Delay50N delayR, 0x03
	
	goto	DETECTIONS
	
CHECK_TIMEOUT
    swapf	OP_sec, 0; 100's seconds
    movwf	temp
    movlw	0x0f
    andwf	temp
    movlw	d'0'
    subwf	temp
    bz	EXIT_EXE	; if 100 second, continue to check for 150s then 120s.

    return
	
DETECTIONS
    ;	movlw	d'2'
;	movwf	STOP_CONDITION	; timeout stop, saved in eeprom as 2
;	movff	OP_sec, temp	; 10's seconds
;	movlw	0x0f
;	andwf	temp
;	movlw	d'2'
;	cpfslt	temp, 0
;	call	CHECK_TIMEOUT	; if 150 second, terminate
    
	; displaying the execution time in seconds
	call	LCD_L2
	swapf	OP_sec, 0	; 100's seconds
	movwf	temp
	movlw	0x0f
	andwf	temp
	movff	temp, WREG
	addlw	0x30
	call	WR_DATA

	movff	OP_sec, temp	; 10's seconds
	movlw	0x0f
	andwf	temp		; Temp = lower nibble of Op_sec
	movff	temp, WREG	; W = Temp
	addlw	0x30		; Convert to ASCII
	call	WR_DATA
	
	swapf	OP_INT, WREG	;1's seconds
	movwf	temp
	movlw	0x0f
	andwf	temp
	movff	temp, WREG
	addlw	0x30
	call	WR_DATA

	movlw	0x73		; Write 's'
	call	WR_DATA
	
	movff	PORTA, temp
	btfsc	temp, 4
	bra	DETECTIONS

	bra	EXIT_EXE
	
;	Delay50N delayR, 0x28
;	call	READ_ARDUINO
;	movwf	DETECTION_VAL
;	bra DETECTIONS
;	
;	bra DETECTIONS

	; first check if there was a bottle detected, if so go to COLLECTIONS_STEP
	movlw	d'5'
	subwf	DETECTION_VAL, 0
	bz	DETECTIONS
	
	; okay we have a bottle, first increment total count then see what it is
	incf	TOTAL_BOTTLES
	
	; yop no cap
	movlw	d'4'
	subwf	DETECTION_VAL, 0
	bz	INC_YOPNOCAP
	
	; yop with cap
	movlw	d'3'
	subwf	DETECTION_VAL, 0
	bz	INC_YOPCAP
	
	; eska no cap
	movlw	d'2'
	subwf	DETECTION_VAL, 0
	bz	INC_ESKANOCAP
	
	; eksa with cap
	movlw	d'1'
	subwf	DETECTION_VAL, 0
	bz	INC_ESKACAP
	
	; edge case, cant determine bottle so check if done sorting
	bra	DETECTIONS
	
INC_YOPNOCAP
	incf	YOP_NOCAP
	bra	DETECTIONS
	
INC_YOPCAP
	incf	YOP
	bra	DETECTIONS
	
INC_ESKANOCAP
	incf	ESKA_NOCAP
	bra	DETECTIONS
	
INC_ESKACAP
	incf	ESKA
	bra	DETECTIONS
	
CHECK_DONE
	; Challenging:	logic to figure out when the machine is done sorting 
	;		if the TOTAL_BOTTLES count is less than 10. 
	
	; OPTIMAL/MAX QUALIFIED TIME: 
	; if the execution time exceeds the optimal threshold of 120s, check for 
	; qualified run and then stop. if the time exceeds the max threshold of 
	; 150s then stop.
	
	; NUMBER: 
	; if the total bottle count is 10, then we are done (most basic end condition)
	clrf	STOP_CONDITION	; regular stop, saved in eeprom as 0
	movlw	d'10'
	subwf	TOTAL_BOTTLES, 0
	bz	EXIT_EXE
	
	bra	DETECTIONS
	; here we know that bottles < 10 and 100 < time < 150, so check for qualified run
	; qualified run has at least 4 bottles, with 1 of each different kind
	movlw	d'3'
	cpfsgt	TOTAL_BOTTLES
	goto	DETECTIONS

	movlw	d'0'
	cpfsgt	YOP_NOCAP, 0
	goto	DETECTIONS
	
	movlw	d'0'
	cpfsgt	YOP, 0
	goto	DETECTIONS
	
	movlw	d'0'
	cpfsgt	ESKA_NOCAP, 0
	goto	DETECTIONS
	
	movlw	d'0'
	cpfsgt	ESKA, 0
	goto	DETECTIONS
	
	; finally, if we get here then consider the termination optimized.
	movlw	d'3'
	movwf	STOP_CONDITION	; optimized stop, saved in eeprom as 3
	goto	EXIT_EXE
	
EXIT_EXE
	; stop timer and save the execution time to eeprom
	bcf	T0CON, TMR0ON
	call	SAVE_EXE_TIME
	call	ClrLCD
	Display	SAVE
		
	; Clear inExecution flag to stop '*' interrupts
	clrf	inExecution
		
	; turn off the dc motor
	movlw	d'0'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	
	Delay50N delayR, 0x05
	; request eska
	movlw	d'2'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	Delay50N delayR, 0x05
	; get eska
	call	READ_ARDUINO
	movwf	ESKA
	
	Delay50N delayR, 0x05
	; request eska without a cap
	movlw	d'3'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	Delay50N delayR, 0x05
	; get eska without a cap
	call	READ_ARDUINO
	movwf	ESKA_NOCAP
	
	Delay50N delayR, 0x05
	; request yop
	movlw	d'4'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	Delay50N delayR, 0x05
	; get yop
	call	READ_ARDUINO
	movwf	YOP
	
	Delay50N delayR, 0x05
	; request eska
	movlw	d'5'
	movff	WREG, databyte
	call	WRITE_ARDUINO
	Delay50N delayR, 0x05
	; get yop without a cap
	call	READ_ARDUINO
	movwf	YOP_NOCAP
	
	call    SaveData
	goto	LOG

;*******************************************************************************
; sorting statistics log mode
;*******************************************************************************
	
LOG
	clrf	inStandby
	call	ClrLCD
	Display Log1

	; display most recent run data
	movlw	d'10'
	movwf	L_EE
	call	DisplayExeTime

	call	LCD_L2
	movlw	d'13'		    ; sorting stats begin at bit 14 in eeprom
	movwf	L_EE
	call	DispOpSort

HOLD_LOG
	call	READ_KEY
	ChangeMode  keyB, LOG_INFO
	ChangeMode  key0, STANDBY
	bra	HOLD_LOG

LOG_INFO
	call	ClrLCD
	clrf	L_EE
	call	DispOpRTC
	
	call	LCD_L2
	Display LogInfo2
	clrf	H_EE
	clrf	L_EE
HOLD_LOG_INFO
	call	READ_KEY
	ChangeMode key0, LOG
	bra	HOLD_LOG
	
;*******************************************************************************
; permanent logs
;*******************************************************************************

PERM_LOG
	setf	inStandby
	call	ClrLCD
	Display	PermLog1
	call	LCD_L2
	Display	PermLog2
	
HOLD_PERM_LOG
	call	READ_KEY
	ChangeMode  key0, STANDBY
	ChangeMode  key1, PLOG
	ChangeMode  key2, PLOG
	ChangeMode  key3, PLOG
	ChangeMode  key4, PLOG
	ChangeMode  key5, PLOG
	ChangeMode  key6, PLOG
	ChangeMode  key7, PLOG
	ChangeMode  key8, PLOG
	ChangeMode  key9, PLOG
	bra HOLD_PERM_LOG
	
PLOG
	clrf	inStandby
	call	ClrLCD
	Display	PermLog1
	
	; find the key press value
	movff	KEY, WREG
	incf	WREG	    ; W = KEY+1
	movwf	temp
	
	rrncf	WREG	    ; division by 4
	bcf	WREG, 7	    ; clear last bit, overflow from rotate
	rrncf	WREG	    
	bcf	WREG, 7
	subwf	temp	    ; temp = (KEY+1) - (KEY+1)/4
	movff	temp, WREG
	movff	temp, temp_KEY
	mullw	d'21'	    ; indexed as mutliples of 21

displayNum
	movff	temp_KEY, WREG
	addlw	0x30
	call	WR_DATA
	movlw	":"
	call	WR_DATA
	movlw	" "
	call	WR_DATA
	
	movff	PRODL, L_EE
	movlw	d'0'
	addwf	L_EE
	READEE	OP_sec, H_EE, L_EE
	incf	L_EE
	READEE	OP_INT, H_EE, L_EE
	incf	L_EE
	call	DisplayExeTime
	movff	PRODL, L_EE
	
	; reset eeprom to beginning
	call	LCD_L2
	movff	PRODL, L_EE
	call	DispOpRTC
	
HOLD_PLOG
	call	READ_KEY
	ChangeMode  keyA, PLOG_DETAILS
	ChangeMode  key0, PERM_LOG	; back to perm log menu
	bra HOLD_PLOG
	
PLOG_DETAILS
	call	ClrLCD
	
	Display	Log1
	call	DisplayExeTime
	
	call	LCD_L2
	call	DispOpSort
	
HOLD_PLOG_DETAILS
	call	READ_KEY
	ChangeMode  key0, PERM_LOG	; back to perm log menu
	bra HOLD_PLOG_DETAILS
	
;*******************************************************************************
; pc interface
;*******************************************************************************

PC_MODE
	setf	inStandby
	call	ClrLCD
	Display	PC1
	call	LCD_L2
	Display	PC2
	movff	PCL, PC_PCL
	movff	PCLATH, PC_PCLATH
	movff	PCLATU, PC_PCLATU
	
HOLD_PC
	call	READ_KEY
	ChangeMode  key0, STANDBY
	ChangeMode  keyH, PC_TRANSFER
	bra	HOLD_PC

PC_TRANSFER
	clrf	inStandby
	setf	transferring
	call	ClrLCD
	Display	PCTransfer
	call	DataUSART
	clrf	transferring
	goto	STANDBY
	
;*******************************************************************************
; subroutines
;*******************************************************************************

SaveData
	movff	    ESKA, WREG
	addlw	    0x30
	WriteEE	    WREG, H_EE, L_EE
	incf	    L_EE
	
	movff	    ESKA_NOCAP, WREG
	addlw	    0x30
	WriteEE	    WREG, H_EE, L_EE
	incf	    L_EE
	
	movff	    YOP, WREG
	addlw	    0x30
	WriteEE	    WREG, H_EE, L_EE
	incf	    L_EE
	
	movff	    YOP_NOCAP, WREG
	addlw	    0x30
	WriteEE	    WREG, H_EE, L_EE
	incf	    L_EE
	
	movff	    STOP_CONDITION, WREG
	addlw	    0x30
	WriteEE	    WREG, H_EE, L_EE
	incf	    L_EE
	
	movlw	    d'0'
	movwf	    H_EE
	movlw	    d'168'		; take d'168' as last
	movwf	    L_EE
	movlw	    d'0'
	movwf	    tempH_EE
	movlw	    d'220'		; temp for low constant
	movwf	    tempL_EE
	
	clrf	    counter2
	clrf	    counter
	
ShiftLoop
	incf		counter
	READEE		REG_EE, H_EE, L_EE
	movlw		d'21'
	addwf		L_EE
	WriteEE		REG_EE, H_EE, L_EE
	movlw		d'20'
	subwf		L_EE
	movlw		d'21'
	cpfseq		counter
	goto		ShiftLoop
	
	; Set EEPROM address to the previous 21 byte block, and reset temp address
	movlw		d'42'
	subwf		L_EE
	movlw		d'220'
	movwf		tempL_EE
	clrf		counter
	incf		counter2
	movlw		d'9'
	cpfseq		counter2	; Skip if 9 shifts were made
	goto		ShiftLoop

	return
	
RTC_INIT
	; set sda and scl to high
	bcf	PORTC, 4
	bcf	PORTC, 3
	bsf	TRISC, 4
	bsf	TRISC, 3
	
	call	INIT_RTC
	
	;call	SET_TIME
return
	
SAVE_TIME
	rtc_read    0x02	; hours	    ; need to call this twice idk y
	rtc_read    0x02	; hours
	call	WriteEE_RTC
	rtc_read    0x01	; minutes
	call	WriteEE_RTC
	rtc_read    0x06	; years
	call	WriteEE_RTC
	rtc_read    0x05	; months
	call	WriteEE_RTC
	rtc_read    0x04	; days
	call	WriteEE_RTC
return
	
SAVE_EXE_TIME
	
	swapf	OP_sec, WREG	; 100's seconds
	movwf	temp
	movlw	0x0f
	andwf	temp
	movff	temp, WREG
	addlw	0x30	
	WriteEE	WREG, H_EE, L_EE
	incf	L_EE
	
	movff	OP_sec, temp	; 10's seconds
	movlw	0x0f
	andwf	temp		; Temp = lower nibble of Op_sec
	movff	temp, WREG	; W = Temp
	addlw	0x30		; Convert to ASCII
	WriteEE	WREG, H_EE, L_EE
	incf	L_EE
	
	swapf	OP_INT, WREG	;1's seconds
	movwf	temp
	movlw	0x0f
	andwf	temp
	movff	temp, WREG
	addlw	0x30
	WriteEE	WREG, H_EE, L_EE
	incf	L_EE
return
	
		
DISPLAY_RTC
	; display data in this format hh/minmin/yy yy/mm/dd
	
	rtc_read    0x02	    ; 0x02 from DS1307 - hours
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
	movlw	"h"
	call	WR_DATA
	
	rtc_read    0x01	    ; 0x01 from DS1307 - minutes
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
	
	rtc_read    0x00	    ; 0x00 from DS1307 - seconds
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
	movlw	" "
	call	WR_DATA
	
	rtc_read    0x06	    ; 0x06 from DS1307 - years
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
	movlw	"/"
	call	WR_DATA
	
	rtc_read    0x05	    ; 0x05 from DS1307 - months
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
	movlw	"/"
	call	WR_DATA
	
	rtc_read    0x04	    ; 0x04 from DS1307 - days
	movff	tens_digit,WREG
	call	WR_DATA
	movff	ones_digit,WREG
	call	WR_DATA
return
	
SET_TIME
	rtc_resetAll
	
	rtc_set	0x00,	b'10000000'

	rtc_set	0x06,	b'00010111'		; year 17
	rtc_set	0x05,	b'00000100'		; month 04
	rtc_set	0x04,	b'00000110'		; day of month 06
	rtc_set	0x02,	b'00100010'		; hours 22
	rtc_set	0x01,	b'00010101'		; minutes 13
	rtc_set	0x00,	b'00000000'		; seconds 0
return
	
INIT_USART
	
	bsf	TRISC, 7	; set RC7=USART RC
	bcf	TRISC, 6	; clear RC6=USART TX

	movlw	15		; baud rate 9600
	movwf	SPBRG
	clrf	TXSTA
	
	clrf	RCSTA
	bsf	RCSTA,SPEN	; asynchronous serial port enable
	bsf	RCSTA,CREN	; continous receive
	
	bsf	TXSTA, TXEN	; transmit enabled
	return

DataUSART
	movlw	0x02
	call	USART_WAIT
	movlw	0x0D
	call	USART_WAIT
	
	clrf	counter		; used to cycle through past 9 eeprom logs
	clrf	counter2	; used to transmit all 18 bits of saved data
	
TRANSFER_LOGS
	movlw	d'21'		; logs are 21 bits apart
	mulwf	counter		; log number one starts at eeprom address 0
	movff	PRODL, L_EE	; low address now points to count X 21
	clrf	counter2
	
TRANSFER_DATA	
	READEE	WREG, H_EE, L_EE
	call	USART_WAIT	; transfer bit
    	incf	L_EE
	
	incf	counter2
	movlw	d'18'		; each perm log has 18 bits of saved data, so 
	cpfseq	counter2	; see if we're at 18. if so, we're done reading
	bra TRANSFER_DATA	; this permanent log and onto next.
	
	call	USART_LINE	; new line because a e s t h e t i c
	
	incf	counter
	movlw	d'8'		; we save up to the last 9 permanent logs
	cpfseq	counter
	bra	TRANSFER_LOGS
	
	call	LCD_L2
	Display	PCComplete
	Delay50N delayR, 0x28	
	return			; done transmitting data
	
USART_WAIT
	movwf	TXREG
	btfss	TXSTA, 1
	goto	$-2
	return

USART_LINE
	movlw	0x0A
	call	USART_WAIT
	movlw	0x0D
	call	USART_WAIT
	return
	
Delay5us
	movlw	d'120'
	movwf	d5us

loop5us
	decfsz	d5us
	goto	loop5us
	nop
	return

Delay200us
	movlw	0xA4
	movwf	d200us

loop200us
	decfsz	d200us
	goto	loop200us
	nop
	return

Delay50ms
	movlw	0xF9
	movwf	d50ms

loop50ms
	call	Delay200us
	decfsz	d50ms

Cycles
	goto	loop50ms
	call	Delay200us
	return

WriteEE_RTC
	WriteEE	tens_digit, H_EE, L_EE
	incf	L_EE
	WriteEE ones_digit, H_EE, L_EE
	incf	L_EE
	return

ClearEEPROM_21
	clrf	H_EE
	clrf	L_EE
	setf	clear_EE
	clrf	counter
ClearNext
	WriteEE		clear_EE, H_EE, L_EE
	incf		counter
	incf		L_EE
	movlw		d'21'
	cpfseq		counter
	goto		ClearNext
	clrf		H_EE
	clrf		L_EE
	return
	
READ_KEY
	btfss	PORTB, 1	; check for keypad interrupt
	goto	READ_KEY
	swapf	PORTB, W
	andlw	0x0F
	movwf	KEY
	btfsc	PORTB, 1
	goto	$-2
	return

READ_KEY_TIME
	; display the time
	; call	    DISPLAY_RTC	

	btfss	    PORTB, 1	; keypad interrupt
	goto	    READ_KEY_TIME
	swapf	    PORTB, 0	; copy PORTB7:4 to W3:0
	andlw	    0x0F	; only want W3:0
	movwf	    KEY		; write this value to the KEY
	btfsc	    PORTB, 1	; wait for release
	goto	    $-2		; go back one instruction
	return

DisplayExeTime
	READEE	REG_EE, H_EE, L_EE
	movlw	0xFF
	cpfseq	REG_EE
	goto	NoSkipDispExeTime
	Display NoData
	movlw	0xFF
	cpfslt	REG_EE
	goto	SkipDispExeTime
	return

NoSkipDispExeTime
	READEE	WREG, H_EE, L_EE
	call	WR_DATA
	incf	L_EE
	
	READEE	WREG, H_EE, L_EE
	call	WR_DATA
	incf	L_EE
	
	READEE	WREG, H_EE, L_EE
	call	WR_DATA
	incf	L_EE

	movlw	0x73		; Write 's'
	call	WR_DATA
	call	LCD_L2
SkipDispExeTime
	return

DispOpSort
	READEE	REG_EE, H_EE, L_EE
	movlw	0xff
	cpfseq	REG_EE
	goto	NoSkipDispOpSort
	Display	NoData
	movlw	0xff
	cpfslt	REG_EE
	goto	SkipDispOpSort

NoSkipDispOpSort
	movlw	"a"
	call	WR_DATA
	
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE
	
	movlw	" "
	call	WR_DATA
	
	movlw	"b"
	call	WR_DATA
	
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE
	
	movlw	" "
	call	WR_DATA
	
	movlw	"c"
	call	WR_DATA
	
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE
	
	movlw	" "
	call	WR_DATA
	
	movlw	"d"
	call	WR_DATA
	
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE

SkipDispOpSort
	return
	
DispOpRTC
	READEE	REG_EE, H_EE, L_EE
	movlw	0xff
	cpfseq	REG_EE
	goto	NoSkipDispOpRTC
	Display	NoData
	movlw	0xff
	cpfslt	REG_EE
	goto	SkipDispOpRTC

NoSkipDispOpRTC
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE
	
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE

	movlw	"h"
	call	WR_DATA
	
	call	DispOpRTC_Helper
	
	movlw	" "
	call	WR_DATA
	
	call	DispOpRTC_Helper
	movlw	"/"
	call	WR_DATA

	call	DispOpRTC_Helper
	movlw	"/"
	call	WR_DATA
	call	DispOpRTC_Helper
SkipDispOpRTC
		return
DispOpRTC_Helper
	READEE		REG_EE, H_EE, L_EE
	movff		REG_EE, WREG
	call		WR_DATA
	incf		L_EE
	READEE		REG_EE, H_EE, L_EE
	movff		REG_EE, WREG
	call		WR_DATA
	incf		L_EE
	return
end