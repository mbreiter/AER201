;*******************************************************************************
;
;    Filename: main.asm
;    Date: 2 Feb 2017
;    File Version: 1.0
;    Author: Matthew Reiter
;    Course: AER201
;    Description: Bottle sorting machine
;*******************************************************************************
; configuration settings
;*******************************************************************************

list    P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC
#include <p18f4620.inc>
#include <lcd.inc>
#include <i2c.inc>
#include <rtc.inc>

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
	temp_ISR
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
	READ_EE
	last_log
	startHt
	startHo
	startMt
	startMo
	endHt
	endHo
	endMt
	endMo
	LED_Count
	SkipCount
	exe_sec
	exe_int
;	tens_digit
;	ones_digit
	timer_H
	timer_L
	convert_buffer
	transferring
	PC_PCL
	PC_PCLATH
	PC_PCLATU
	TIMCNT
	CPCNT
	TDATA
    endc

;*******************************************************************************
; tables
;*******************************************************************************
    Welcome	db	    "botL", 0
    Team	db	    "mr hl hg", 0
    StandBy	db	    "Standby", 0
    Log1	db	    "Time:",0
    Log2	db	    "12:00 2/3/14", 0
    LogInfo1	db	    "Saved:", 0
    LogInfo2	db	    "Back <0>", 0
    PermLog1	db	    "Permanent Logs", 0
    PermLog2	db	    "<1> ... <9>", 0	
    Exe1	db	    "Sorting...", 0
    Exe2	db	    "END <*>", 0
    PC1		db	    "PC Interface", 0
    PC2		db	    "Begin <#>", 0
    PCTransfer	db	    "Transferring...", 0
    Stopped	db	    "Stopped", 0
    Op_Time	db	    "Time: ", 0
    SAVE	db	    "Saving...", 0
    Safety	db	    "Safety check...", 0
    NoData	db	    "N/A", 0
    PCLog1	db	    "Time and Date:", 0
	
;*******************************************************************************
; macros
;*******************************************************************************

TransferTable macro table
	local loop
	
	movlw	upper table
	movwf	TBLPTRU
	movlw	high table
	movwf	TBLPTRH
	movlw	low table
	movwf	TBLPTRL
	tblrd*
	movf	TABLAT, W
loop
	call	USART_WAIT
	tblrd+*
	movf	TABLAT, W
	bnz	loop
	endm
	
ConfigLCD   macro
          movlw     b'00101000'    ; 4 bits, 2 lines,5X7 dots seems to work best instead of the above setting
          call      WR_INS

          movlw     b'00001100'    ; display on/off
          call      WR_INS
         ; movlw     B'00000110'    ; Entry mode
         ; call      WR_INS
          movlw     b'00000001'    ; Clear ram
          call      WR_INS
	  endm
	
movMSB	macro	port
	andlw	0xF0
	iorwf	port, f
	iorlw	0x0F
	andwf	port, f
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

DisplayLog  macro   addrH, addrL
	    local   Again, Finish
	    clrf    SkipCount
	    clrf    MAX_EE

	    ; Check if there is No Data first
	    READEE	REG_EE, addrH, addrL
	    movlw	0xFF
	    cpfseq	REG_EE
	    goto	Again
	    Display	NoData
	    goto	Finish
Again
	    ; Put a space every 3 Writes
	    movlw	d'3'
	    cpfslt	SkipCount
	    cpfslt	MAX_EE
	    goto	Finish

	    ; read EEPROM
	    READEE	REG_EE, addrH, addrL
	    btfsc	REG_EE, 7	; if bit 7 set then done
	    goto	Finish
Finish
	    endm
	    
;TransferLog macro   addrH, addrL
	

incf_BCD    macro   BCD
	local ones, endBCD
	movff	BCD, temp
	
	;process ones digit
	movlw	0x0F
	andwf	temp
	movlw	d'9'
	
	;skip if ones digit is 9
	cpfseq	temp
	goto	ones
	
	movlw	b'00010000'
	addwf	BCD
	movlw	0x0F
	andwf	BCD
	movlw	0xA0
	cpfslt	BCD
	
	clrf	BCD
	goto	endBCD
	
ones
	incf	BCD
	goto	endBCD
endBCD
	nop
	endm

WriteRTC    macro
	movf	tens_digit, W
	call	WR_DATA
	movf	ones_digit, W
	call	WR_DATA
	endm

WriteEE	macro   word, addH, addL
	movff   addH, EEADRH    ; High address
	movff   addL, EEADRH    ; Low address
	movff   word, EEDATA    ; set word

	btfsc   EECON1, WR	    ; test if WR=0, skip if so
	bra	$-2

	bcf	EECON1, EEPGD   ; Point to data memory
	bcf	EECON1, CFGS    ; Access EEPROM
	bsf	EECON1, WREN    ; enable write
	bcf	EECON1, GIE	    ; disable interrupts during write
	bcf	PIR2, EEIF

	movlw   0x55
	movwf   EECON2	    ; write 55h
	movlw   0xAA
	movwf   EECON2	    ; write 0xAA
	bsf	EECON1, WR	    ; begin write
	btfsc   EECON1, WR
	bra	$-2

	bsf	EECON1, GIE	    ; enable interrupts after write
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
    swapf   temp_W, W               ; restore W first
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

ISR_HIGH
	saveContext

	;reset timer
	movlw	timer_H
	movwf	TMR0H
	movlw	timer_L - 9
	movwf	TMR0L
	
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
	btfss		INTCON3, INT1IF			; If KEYPAD interrupt, skip return
	goto		END_ISR_LOW

	;Check operation status
;	movlw		0xFF					; If in operation, skip return
;	cpfseq		InExecution
;	goto		END_ISR_LOW

	; Process KEY
	swapf		PORTB, W				; Read PORTB<7:4> into W<3:0>
	andlw		0x0F
	movwf		KEY_ISR					; Put W into KEY_ISR
;	movlw		keyS					; Put keyStar into W to compare to KEY_ISR
;	cpfseq		KEY_ISR					; If key was '*', skip return
	goto		END_ISR_LOW

	; Reset program counter
	clrf		TOSU
	clrf		TOSH
	clrf		TOSL

END_ISR_LOW
	bcf			INTCON3, INT1IF         ; Clear flag for next interrupt
	restoreContext
	
	movwf	temp_W		    ; save current W
	movff	STATUS, temp_S	    ; save status

	; ISR_Low routines
	movff	temp_S, STATUS	    ; retreive status
	swapf	temp_W, f
	swapf	temp_W, w	    ; restore W
	retfie

;*******************************************************************************
; main
;*******************************************************************************
INIT
	; i/o
	movlw	b'00000000'
	movwf	TRISA
	movlw	b'11111111'
	movwf	TRISB		    ; keypad
	movlw	b'10111111'
	movwf	TRISC
	
	movlw	b'00000000'
	movwf	TRISD
	movlw	b'00000111'
	movwf	TRISE

	; analog/digital pins
	movlw	b'00001111'     ; Set all AN pins to Digital
	movwf   ADCON1

	; clear ports
        clrf	LATA
        clrf	LATB
	bcf	TRISC, SCL
	bcf	TRISC, SDA
        clrf	LATC
        clrf	LATD
	clrf	LATE
	
	; initializations
	
	call	InitLCD
	ConfigLCD
	
	call	RTC_INIT
	call	Delay50ms
	
	call	INIT_USART

	movlw	b'00001000'
	movwf	T0CON

	; interrupts
	clrf	RCON
	clrf	INTCON
	clrf	INTCON2
	clrf	INTCON3
	bsf	RCON, IPEN	    ; priority mode
	bsf	INTCON, GIEH	    ; permit global interrupts
	bsf	INTCON, GIEL
	bsf	INTCON2, INTEDG1    ; INTEDG1 on rising edge
	bsf	INTCON, TMR0IE
	bsf	INTCON2, TMR0IP	    ; set to high priority
	bsf	INTCON3, INT1IE
	bcf	INTCON3, INT1IP

	clrf	H_EE
	clrf	L_EE
	clrf	tens_digit
	clrf	ones_digit
	
	movlw     b'11110010'    ; Set required keypad inputs
        movwf     TRISB
	
	Display	Welcome
	call LCD_L2
	Display	Team
	Delay50N delayR, 0x3C

;*******************************************************************************
; standby mode
;*******************************************************************************
STANDBY
	call	ClrLCD
	Display	StandBy

HOLD_STANDBY
	call	READ_KEY_TIME
	ChangeMode keyA, EXECUTION
	ChangeMode keyB, LOG
	ChangeMode keyC, PERM_LOG
	ChangeMode keyD, PC_MODE
	bra	HOLD_STANDBY

;*******************************************************************************
; execution mode
;*******************************************************************************

EXECUTION
	; save the starting time
	call	START_TIME
	
	; display
        setf	    inExecution
	call	    ClrLCD
	Display	    Exe1
	call	    LCD_L2
	Display	    Exe2
	
	; start timer
	movlw	    timer_H				; 1
	movwf	    TMR0H
	movlw	    timer_L				; 1
	movwf	    TMR0L				; 1
	bsf	    T0CON, TMR0ON			; Turn on timer
	call	    ClearEEPROM_21
	
	; initialize variables
	clrf	    OP_sec
	clrf	    OP_INT

HOLD_EXE
	call	    READ_KEY
	ChangeMode  keyS, EXIT_EXE
	bra	    HOLD_EXE

EXIT_EXE
	WriteEE	    OP_sec, H_EE, L_EE
	incf	    L_EE
	WriteEE	    OP_INT, H_EE, L_EE
	incf	    L_EE
	

	
	; Clear InOperation flag to stop '*' interrupts
	clrf	    inExecution
        goto        SaveData

SaveData
	call	    ClrLCD
	Display	    SAVE
	movlw	    d'0'
	movwf	    H_EE
	movff	    last_log, L_EE
	movlw	    d'0'
	movwf	    tempH_EE
	movff	    tempL_EEC, tempL_EE
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
	
	; Set EEPROM address to the previous 21 byte block, and reset TempEEPROM address
	movlw		d'42'
	subwf		L_EE
	movff		tempL_EEC, tempL_EE
	clrf		counter
	incf		counter2
	movlw		d'9'
	cpfseq		counter2					; Skip if 9 shifts were made
	goto		ShiftLoop
	
	; Finish Saving Data
	; Stop Timer and goto OpLog
	bcf		T0CON, TMR0ON
	movlw		d'9'
	movwf		L_EE
	WriteEE		OP_sec, H_EE, L_EE
	incf		L_EE
	WriteEE		OP_INT, H_EE, L_EE
	incf		L_EE
	goto		LOG

;*******************************************************************************
; sorting statistics log mode
;*******************************************************************************
	
LOG
	call	ClrLCD
	Display Log1

	; display most recent run data
	movlw	d'9'
	movwf	L_EE
	READEE	OP_sec, H_EE, L_EE
	incf	L_EE
	READEE	OP_INT, H_EE, L_EE
	incf	L_EE
	call	DisplayTime

	call	LCD_L2
	clrf	L_EE
	call	DispOpRTC

HOLD_LOG
	call	READ_KEY
	ChangeMode  keyB, LOG_INFO
	ChangeMode  key0, STANDBY
	bra	HOLD_LOG

LOG_INFO
	call	ClrLCD
	Display	LogInfo1
	call	LCD_L2
	Display LogInfo2
HOLD_LOG_INFO
	call	READ_KEY
	ChangeMode key0, LOG
	bra	HOLD_LOG
	
;*******************************************************************************
; permanent logs
;*******************************************************************************

PERM_LOG
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
	call	DisplayTime
	movff	PRODL, L_EE
	
	; reset eeprom to beginning
	call	LCD_L2
	movff	PRODL, L_EE
	call	DispOpRTC
	
HOLD_PLOG
	call	READ_KEY
	ChangeMode  key0, PERM_LOG	; back to perm log menu
	bra HOLD_PERM_LOG
	
;*******************************************************************************
; pc interface
;*******************************************************************************

PC_MODE
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
	setf	transferring
	call	ClrLCD
	Display	PCTransfer
	;call	DataUSART
	clrf	transferring
	bra	HOLD_PC
	
;*******************************************************************************
; subroutines
;*******************************************************************************

RTC_INIT
	; set sda and scl to high-z
	bcf	PORTC, 4
	bcf	PORTC, 3
	bsf	TRISC, 4
	bsf	TRISC, 3

	call	i2c_common_setup
	;call	SET_TIME
return
	
START_TIME
	rtc_read    0x02	    ; hour
	movf	tens_digit, WREG
	movwf	startHt
	movf	ones_digit, WREG
	movwf	startHo
	
	rtc_read    0x01	    ; minute
	movf	tens_digit, WREG
	movwf	startMt
	movf	ones_digit, WREG
	movwf	startMo
return
	
END_TIME
	rtc_read    0x02	    ; hour
	movf	tens_digit, WREG
	movwf	endHt
	movf	ones_digit, WREG
	movwf	endHo
	
	rtc_read    0x01	    ; minute
	movf	tens_digit, WREG
	movwf	endMt
	movf	ones_digit, WREG
	movwf	endMo
return
	
	
DISPLAY_RTC
	
	; display data in this format hh/minmin/yy yy/mm/dd
	rtc_read    0x02	    ; 0x02 from DS1307 - hours
	movf	tens_digit,WREG
	call	WR_DATA
	movf	ones_digit,WREG
	call	WR_DATA
	movlw	"h"
	call	WR_DATA
	
	rtc_read    0x01	    ; 0x01 from DS1307 - minutes
	movf	tens_digit,WREG
	call	WR_DATA
	movf	ones_digit,WREG
	call	WR_DATA
	movlw	" "
	call	WR_DATA
	
	rtc_read    0x06	    ; 0x06 from DS1307 - years
	movf	tens_digit,WREG
	call	WR_DATA
	movf	ones_digit,WREG
	call	WR_DATA
	movlw	"/"
	call	WR_DATA
	
	rtc_read    0x05	    ; 0x06 from DS1307 - months
	movf	tens_digit,WREG
	call	WR_DATA
	movf	ones_digit,WREG
	call	WR_DATA
	movlw	"/"
	call	WR_DATA
	
	rtc_read    0x04	    ; 0x06 from DS1307 - days
	movf	tens_digit,WREG
	call	WR_DATA
	movf	ones_digit,WREG
	call	WR_DATA
return
	
SET_TIME
	rtc_resetAll
	
	rtc_set	0x00,	b'10000000'

	rtc_set	0x06,	b'00010111'		; Year 17
	rtc_set	0x05,	b'00000010'		; Month 2
	rtc_set	0x04,	b'00010000'		; Date 10
	rtc_set	0x02,	b'00011000'		; Hours 18
	rtc_set	0x01,	b'00100111'		; Minutes 27
	rtc_set	0x00,	b'00000000'		; Seconds 0
return
	
INIT_USART
	movlw	15	; baud rate 9600
	movwf	SPBRG
	clrf	TXSTA
	
	clrf	RCSTA
	bsf	RCSTA,SPEN	; asynchronous serial port enable
	bsf	RCSTA,CREN	; continous receive
	
	bsf	TXSTA, TXEN	; transmit enabled
	return

;DataUSART
;	
;	movlw	0x02
;	call	USART_WAIT
;	movlw	0x0D
;	call	USART_WAIT
;	
;	movlw	d'21'	    ; start of permanent logs
;	movff	w, L_EE
;	TransferTable	PCLog1
;	
;	; display time here
;;	rtc_read	0x02
;;		movf        tens_digit, W
;;        andlw       b'00000001'
;;        addlw       0x30
;;        call        TransmitWaitUSART
;;        movf        ones_digit, W
;;        call        TransmitWaitUSART
;;		movlw		":"
;;		call		TransmitWaitUSART
;;		; Dispay minutes
;;		rtc_read	0x01
;;		call        SendRTC_USART
;;		; Dispay AM/PM
;;		rtc_read	0x02
;;        movlw       "P"
;;        btfss       tens_digit, 1
;;        movlw       "A"
;;        call        TransmitWaitUSART
;;        movlw       "M"
;;        call        TransmitWaitUSART
;;		movlw		" "
;;        call        TransmitWaitUSART
;;		; Display month
;;		rtc_read	0x05
;;		call        SendRTC_USART
;;		movlw		0x2F		; ASCII '/'
;;		call		TransmitWaitUSART
;;		; Display day
;;		rtc_read	0x04
;;		call        SendRTC_USART
;;		movlw		0x2F		; ASCII '/'
;;		call		TransmitWaitUSART
;;		; Display year
;;		rtc_read	0x06
;;		call        SendRTC_USART
;	
;	call	USART_LINE
;	call	USART_LINE
;	
;	clrf	counter
;	incf	counter
;
;USART_TRANSFER
;	movlw	d'21'
;	mulwf	counter
;	movff	PRODL, L_EE
;	movlw	d'9'
;	addwf	L_EE
;	READEE	OP_sec, H_EE, L_EE
;	incf	L_EE
;	READEE	OP_INT, H_EE, L_EE
;	incf	L_EE
;	; call	TransferTime
;	
;	; send rtc
;	movlw	0x09
;	call	USART_WAIT
;	movff	PRODL, L_EE
;	;call	TransferRTC
;	call	USART_LINE
;	
;	
;	
;	
;USART_WAIT
;	movwf	TXREG
;	btfss	TXSTA,1
;	goto	$-2
;	return
;
;USART_LINE
;	movlw	0x0A
;	call	USART_WAIT
;	movlw	0x0D
;	call	USART_WAIT
;	return
	
	
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
	clrf		H_EE		; reset EEPROMaddr
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
	call	    LCD_L2	    ; go to second line to print RTC

	; display the time
	call	DISPLAY_RTC

	btfss	    PORTB, 1	; keypad interrupt
	goto	    READ_KEY_TIME
	swapf	    PORTB, W	; copy PORTB7:4 to W3:0
	andlw	    0x0F	; only want W3:0
	movwf	    KEY		; write this value to the KEY
	btfsc	    PORTB, 1	; wait for release
	goto	    $-2		; go back one instruction
	return

DisplayTime
	READEE	REG_EE, H_EE, L_EE
	movlw	0xFF
	cpfseq	REG_EE
	goto	NoSkipDispOpTime
	Display NoData
	movlw	0xFF
	cpfslt	REG_EE
	goto	SkipDispOpTime

NoSkipDispOpTime
	swapf	OP_sec, W
	movwf	temp
	movlw	0x0F
	andwf	temp
	movff	temp, WREG
	addlw	0x30
	call	WR_DATA

	movff	OP_sec, temp	; 1's seconds
	movlw	0x0F
	andwf	temp		; Temp = lower nibble of Op_Seconds
	movff	temp, WREG	; W = Temp
	addlw	0x30		; Convert to ASCII
	call	WR_DATA

	movlw	0x2E		; Write '.'
	call	WR_DATA

	movlw	0x73		; Write 's'
	call	WR_DATA
	call	LCD_L2
SkipDispOpTime
	return

DispOpRTC
	movlw	d'11'
	addwf	L_EE

	READEE	REG_EE, H_EE, L_EE
	movlw	0xFF
	cpfseq	REG_EE
	goto	NoSkipDispOpRTC
	Display	NoData
	movlw	0xFF
	cpfslt	REG_EE
	goto	SkipDispOpRTC

NoSkipDispOpRTC
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	andlw	b'11110001'
	call	WR_DATA
	incf	L_EE
	READEE	REG_EE, H_EE, L_EE
	movff	REG_EE, WREG
	call	WR_DATA
	incf	L_EE

	movlw		":"
	call		WR_DATA
	call DispOpRTC_Helper
	movlw		" "
	call		WR_DATA
	call DispOpRTC_Helper
	movlw		"/"
	call		WR_DATA
	call DispOpRTC_Helper
	movlw		"/"
	call		WR_DATA
	call DispOpRTC_Helper
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

