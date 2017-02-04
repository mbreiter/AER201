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
    
#include <p18f4620.inc>
#include <lcd.inc>
#include <rtc.inc>
    List    P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

    CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
    CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
    CONFIG WDT = ON, WDTPS = 32768
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

; pin configurations
#define	    RS		LATD, 2	    ; LCD
#define	    E		LATD, 3	    ; LCD
    
; timers
	    CODE
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
temp_ISR    equ		0x00

; temporary registers to save W and status
temp_S	    equ		0x01
temp_W	    equ		0x02
counter	    equ		0x03
counter2    equ		0x04

; delays for timers
delayR	    equ		0x28	    ; primary delay register
d5us	    equ		0x29	    ; 5 us delay
d200us	    equ		0x30	    ; 200 us delay
d50ms	    equ		0x31	    ; 50 ms delay
	    	    
; EEPROM
#define	    temp_EE	d'220'
#define	    last_log	d'168'
clear_EE    equ		0x70
H_EE	    equ		0x71
L_EE	    equ		0x72
REG_EE      equ		0x73
tempH_EE    equ		0x74
tempL_EE    equ		0x75
MAX_EE      equ		0x76
READ_EE	    equ		0x77

; pc interface	

;*******************************************************************************
; tables
;*******************************************************************************
Welcome		db	    "botL", 0
Team		db	    "mr hl hg", 0
StandBy		db	    "Standby", 0
OpExe		db	    "Sorting...", 0
OpStop		db	    "Stopped", 0
Op_Time		db	    "Time: ", 0
Op_RTC		db	    "hhmmss, mm/dd/yy", 0
Save		db	    "Saving Logs...", 0
Transfer	db	    "Transferring...", 0
Safety		db	    "Safety check...", 0		
	    
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
	    
WriteEE	    macro   word, addH, addL
	movff   addH, EEADRH    ; High address
	movff   addL, EEADRH    ; Low address
	movff   word, EEDATA    ; set word

	btfsc   EECON1, WR	    ; test if WR=0, skip if so
	bra	    $-2

	bcf	    EECON1, EEPGD   ; Point to data memory
	bcf	    EECON1, CFGS    ; Access EEPROM
	bsf	    EECON1, WREN    ; enable write
	bcf	    EECON1, GIE	    ; disable interrupts during write
	bcf	    PIR2, EEIF

	movlw   0x55
	movwf   EECON2	    ; write 55h
	movlw   0xAA	    
	movwf   EECON2	    ; write 0xAA
	bsf	    EECON1, WR	    ; begin write
	btfsc   EECON1, WR	    
	bra	    $-2

	bsf	    EECON1, GIE	    ; enable interrupts after write
	bcf	 ECON1, WREN	    ; disable writes
	endm

READEE	    macro   file, addH, addL
	movff   addH, EEADRH	    ; read to high
	movff   addL, EEADR	    ; read to low
	bcf	EECON1, EEPGD	    ; point to data memory
	bcf	EECON1, CFGS	    ; access to EEPROM
	bsf	EECON1, RD	    ; read EEPROM
	movff   EEDATA, file	    ; put data into file
	endm

ChangeMode  macro   Key, Next
	endm

;*******************************************************************************
; Reset Vector and ISRs
;*******************************************************************************
	ORG	0x000		    ; processor reset vector
	goto    INIT		    ; go to beginning of program

	ORG	0x008
	goto    ISR_High

	ORG	0x018
	goto    ISR_LOW
    
ISR_High
	movwf	temp_W		    ; save current W
	movff	STATUS, temp_S	    ; save status
	
	; ISR_High routines
	movff	temp_S, STATUS	    ; retreive status
	swapf	temp_W, f
	swapf	temp_W, w	    ; restore W
	retfie

ISR_LOW
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
	movwf	TRISB		    ; keypads
	movlw	b'10111111'
	movwf	TRISC
	movlw	b'00000000'
	movwf	TRISD
	movlw	b'00000111'
	movwf	TRISE
	
	; analog/digital pins
	
	; clear Ports
	clrf	LATA
	clrf	LATB
	clrf	LATC
	clrf	LATD
	clrf	LATE
	
	; initializations
	call	InitLCD
	call	i2c_common_setup
	;call	initRTC
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
	
	Display	Welcome
	Delay50N delayR, 0x3C
	call	ClrLCD
	Display	StandBy
	
;*******************************************************************************
; standby mode
;*******************************************************************************
STANDBY
	bra	STANDBY
WAIT
	
	
;*******************************************************************************
; initializations
;*******************************************************************************

initRTC
	rtc_resetAll
	rtc_set	    0x00, b'00000000' ;0 s
	rtc_set	    0x01, b'00010111' ;21 min
	rtc_set	    0x02, b'00100000' ;18h
	rtc_set	    0x04, b'00000011' ;3rd day
	rtc_set	    0x05, b'00000010' ;February
	rtc_set	    0x06, b'00010001' ;2017
	return

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
	goto loop200us
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
	
	end