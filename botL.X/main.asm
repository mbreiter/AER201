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

    CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
    CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
    CONFIG WDT = OFF, WDTPS = 32768
    CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
    CONFIG STVREN = ON, LVP = OFF, XINST = OFF
    CONFIG DEBUG = ON
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
; timers
#define	    timer_H	0x9E
#define	    timer_L	0x58
OP_sec	    equ	        0x10
OP_INT	    equ		0x11

; keys
key1        equ		d'0'
key2        equ		d'1'
key3        equ		d'2'
keyA        equ		d'3'
key4	    equ		d'4'
key5	    equ		d'5'
key6	    equ	        d'6'
keyB        equ		d'7'
key7	    equ		d'8'
key8	    equ		d'9'
key9        equ	 	d'10'
keyC	    equ		d'11'
keyS        equ		d'12'
key0	    equ		d'13'
keyH	    equ		d'14'
keyD	    equ		d'15'

KEY	    equ		0x50
temp_KEY    equ		0x51
KEY_ISR	    equ		0x52

; temporary variable used in isrs
temp_ISR    equ		0x05

; temporary registers to save W and status
temp_S	    equ		0x01
temp_W	    equ		0x02
counter	    equ		0x03
counter2    equ		0x04
temp	    equ		0x06

; delays for timers
delayR	    equ		0x28	    ; primary delay register
d5us	    equ		0x29	    ; 5 us delay
d200us	    equ		0x30	    ; 200 us delay
d50ms	    equ		0x31	    ; 50 ms delay

; execution
inExecution equ		0x60
	    
; EEPROM
#define	    temp_EE	d'220'
#define	    last_log	d'168'
clear_EE    equ		0x70
H_EE	    equ		0x71
L_EE	    equ		0x72
REG_EE      equ		0x73
tempH_EE    equ		0x74
tempL_EE    equ		0x75
tempL_EEC   equ		0x82
MAX_EE      equ		0x76
READ_EE	    equ		0x77
LED_Count   equ		0x78
SkipCount   equ		0x79

; execution
exe_sec	    equ		0x80
exe_int	    equ		0x81

; pc interface
; rtc
tens_digit  equ		0x90
ones_digit  equ		0x91

;*******************************************************************************
; tables
;*******************************************************************************
Welcome		db	    "botL", 0
Team		db	    "mr hl hg", 0
StandBy		db	    "Standby", 0
Log1		db	    "Time:",0
Log2		db	    "12:00 2/3/14", 0
LogInfo1	db	    "Saved:", 0
LogInfo2	db	    "Back <0>", 0
PermLog1	db	    "Permanent Logs", 0
PermLog2	db	    "<1> ... <9>", 0	
Exe1		db	    "Sorting...", 0
Exe2		db	    "END <*>", 0
PC1		db	    "PC Interface", 0
PC2		db	    "Begin <#>", 0
PCTransfer	db	    "Transferring...", 0
Stopped		db	    "Stopped", 0
Op_Time		db	    "Time: ", 0
SAVE		db	    "Saving...", 0

Safety		db	    "Safety check...", 0
NoData		db	    "N/A", 0

;*******************************************************************************
; macros
;*******************************************************************************

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
	movff	tens_digit, W
	call	WR_DATA
	movff	ones_digit, W
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

	; clear Ports
	clrf	LATA
	clrf	LATB
	clrf	LATC
	clrf	LATD
	clrf	LATE

	; initializations
	call	InitLCD
	call	I2C_Master_INIT
	call	Delay50ms
	;call	initUSART
	;call	initEEPROM

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

;	; set real time clock
;	call	I2C_Master_START
;	movlw	'w'
;	call	WR_DATA
;	call	Delay50ms
;	movlw	b'11010000'
;	call	I2C_Master_WRITE
;	movlw	0x00		    ; memory point to seconds
;	call	I2C_Master_WRITE
;
;	movlw	0x00		    ; set seconds to 0
;	call	I2C_Master_WRITE
;	movlw	0x40		    ; set minutes to 40
;	call	I2C_Master_WRITE
;	movlw	0x15		    ; set hours to 15
;	call	I2C_Master_WRITE
;	movlw	0x5		    ; set day to 5
;	call	I2C_Master_WRITE
;	movlw	0x02		    ; set month to feb
;	call	I2C_Master_WRITE
;	movlw	0x17		    ; set year to 17
;	call	I2C_Master_STOP

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
	call	READ_KEY_RTC
	ChangeMode keyA, EXECUTION
	ChangeMode keyB, LOG
	ChangeMode keyC, PERM_LOG
	;ChangeMode keyD, PC
	bra	HOLD_STANDBY

;*******************************************************************************
; execution mode
;*******************************************************************************

EXECUTION
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
	
;	movlw	    0x02
;	call	    I2C_Master_READ					; Hours
;	call	    WriteEE_RTC
;	
;	movlw	    0x01
;	call	    I2C_Master_READ					; Minutes
;	call	    WriteEE_RTC
;	
;	movlw	    0x05
;	call	    I2C_Master_READ					; Month
;	call	    WriteEE_RTC
;	
;	movlw	    0x04
;	call	    I2C_Master_READ					; Day
;	call	    WriteEE_RTC
;	
;	movlw	    0x06
;	call	    I2C_Master_READ					; Year
;	call	    WriteEE_RTC
	
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
	;Stop Timer and goto OpLog
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
;	ChangeMode  key2, PLOG
;	ChangeMode  key3, PLOG
;	ChangeMode  key4, PLOG
;	ChangeMode  key5, PLOG
;	ChangeMode  key6, PLOG
;	ChangeMode  key7, PLOG
;	ChangeMode  key8, PLOG
;	ChangeMode  key9, PLOG
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
; subroutines
;*******************************************************************************

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
HOLD_KEY
	btfss	PORTB, 1	; check for keypad interrupt
	goto	HOLD_KEY
	swapf	PORTB, W
	andlw	0x0F
	movwf	KEY
	btfsc	PORTB, 1
	goto	$-2
	return

READ_KEY_RTC
HOLD_KEY_RTC
	call	    LCD_L2	; go to second line to print RTC
;	call	    I2C_Master_START
;	movlw	    b'11010000'
;	movlw	    0x00
;	call	    I2C_Master_WRITE
;    	call	    I2C_Master_STOP
;
;	movlw	    0x01	; set ACKDT
;	movwf	    I2C_ACKDT
;
;	movlw	    0x02	; get hours
;	call	    I2C_Master_READ
;	movf	    tens_digit, W
;	andlw	    b'00000001'
;	addlw	    0x30
;	call	    WR_DATA
;	movf	    ones_digit, W
;	call	    WR_DATA
;	movlw	    0x01	; get minutes
;	call	    I2C_Master_READ
;	WriteRTC
;	movlw	    " "		; wow grate formatng very nice. thank you
;	call	    WR_DATA
;	movlw	    0x05	; month
;	call	    I2C_Master_READ
;	WriteRTC
;	movlw	    0x2F	; ascii code for forward slash
;	call	    WR_DATA
;	movlw	    0x04	; get day
;	call	    I2C_Master_READ
;	WriteRTC
;	movlw	    0x2F	; ascii code for forward slash
;	call	    WR_DATA
;	movlw	    0x00	; clear ACKDT
;	movwf	    I2C_ACKDT
;	movlw    0x06		; get year
;	call	    I2C_Master_READ
;	WriteRTC

	btfss	    PORTB, 1	; keypad interrupt
	goto	    HOLD_KEY_RTC
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
