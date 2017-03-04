#include <p18f4620.inc>

;Declare constants for pin assignments (LCD on PORTD)
RS 	equ 2
E 	equ 3

#define		LCD_RS      LATD, 2        ; for v 1.0 used PORTD.3
#define		LCD_E       LATD, 3        ; for v 1.0 used PORTD.2

lcd_tmp		equ		0x68
lcd_d1		equ		0x69
lcd_d2		equ		0x70
temp_lcd	equ		0x71           ; buffer for Instruction
dat		equ		0x72           ; buffer for data
delay1		equ		0x73
delay2		equ		0x74
delay3		equ		0x75

;Helper macros
WRT_LCD macro val
	movlw   val
	call    WrtLCD
	endm
	
;Delay: ~44us
LCD_DELAY macro
	movlw   0x23
	movwf   lcd_d1
	decfsz  lcd_d1,f
	goto    $-2
	endm

	code
	global InitLCD,WrtLCD,ClkLCD,ClrLCD,WR_INS,WR_DATA,LCD_L1,LCD_L2,delay5ms,delay44us

; ****************************************************************************
; GLOBAL SUBROUTINES
; ****************************************************************************
InitLCD
	;bsf PORTD,E     ;E default high	
	;Wait for LCD POR to finish (~15ms)
	call lcdLongDelay
	call lcdLongDelay
	call lcdLongDelay
	;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
	; -> Send b'0011' 3 times
	bcf     PORTD,RS       ;Instruction mode
	movlw   B'00110000'
	call    MovMSB
	call    ClkLCD         ;Finish last 4-bit send (if reset occurred in middle of a send)
	call    ClkLCD         ;Assuming 4-bit mode, set 8-bit mode
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	call    ClkLCD         ;(note: if it's in 8-bit mode already, it will stay in 8-bit mode)
    ;Now that we know for sure it's in 8-bit mode, set 4-bit mode.
	movlw B'00100000'
	call MovMSB
	call ClkLCD
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	;Give LCD init instructions
	WRT_LCD B'00101000' ; 4 bits, 2 lines,5X8 dot
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	WRT_LCD B'00001100' ; display on,cursor,blink
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	WRT_LCD B'00000110' ; Increment,no shift
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	;Ready to display characters
	call    ClrLCD
    bsf     PORTD,RS    ;Character mode
	return

;****************************************
;		Write command to LCD
;		Input  : W
;		output : -
;****************************************
WR_INS
		bcf	LCD_RS	  				; clear Register Status bit
		movwf	temp_lcd			; store instruction
		andlw	0xf0			  	; mask 4 bits MSB
		movwf	LATD			  	; send 4 bits MSB

		bsf	LCD_E					; pulse enable high
		swapf	temp_lcd, 0		; swap nibbles
		andlw	0xF0			  	; mask 4 bits LSB
		bcf	LCD_E
		movwf	LATD			  	; send 4 bits LSB
		bsf	LCD_E					; pulse enable high
		bcf	LCD_E
		call	delay5ms

		return

;***************************************
;		Write data to LCD
;		Input  : W
;		Output : -
;***************************************
WR_DATA
		bcf		LCD_RS					; clear Register Status bit
        movwf   dat					; store character
        movff	dat, WREG
		andlw   0xF0			  	; mask 4 bits MSB
        addlw   4					; set Register Status
        movwf   PORTD			  	; send 4 bits MSB

		bsf		LCD_E					; pulse enable high
        swapf   dat, 0		  	; swap nibbles
        andlw   0xF0			  	; mask 4 bits LSB
		bcf		LCD_E
        addlw   4					; set Register Status
        movwf   PORTD			  	; send 4 bits LSB
		bsf		LCD_E					; pulse enable high
		bcf		LCD_E

		call	delay44us

        return

; WrtLCD: Clock MSB and LSB of W to PORTD<7:4> in two cycles
WrtLCD
	movwf   lcd_tmp ; store original value
	call    MovMSB  ; move MSB to PORTD
	call    ClkLCD
	swapf   lcd_tmp,0 ; Swap LSB of value into MSB of W
    call    MovMSB    ; move to PORTD
    call    ClkLCD

    return

; ClrLCD: Clear the LCD display
ClrLCD
    bcf     PORTD,RS       ;Instruction mode
    WRT_LCD b'00000001'
    call    lcdLongDelay
    return

; ClkLCD: Pulse the E line low
ClkLCD
    ;LCD_DELAY
    bsf PORTD,E
    nop
	;LCD_DELAY   ; __    __
    bcf PORTD,E ;   |__|
	LCD_DELAY
    return

; Change LCD Lines
LCD_L1
		movlw		B'10000000'
		call		WR_INS
		return
LCD_L2
		movlw		B'11000000'
		call		WR_INS
		return
; ****************************************************************************
; EXTRA HELPER SUBROUTINES
; ****************************************************************************
;MovMSB: Move MSB of W to PORTD, without disturbing LSB
MovMSB
    andlw 0xF0
    iorwf PORTD,1
    iorlw 0x0F
    andwf PORTD,1
    return

    ;Delay: ~5ms
lcdLongDelay
    movlw d'80'
    movwf lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz lcd_d2,1
    goto LLD_LOOP
    return

;******************************************************************************
; Delay44us (): wait exactly  110 cycles (44 us)
; <www.piclist.org>

delay44us
		movlw	0x23
		movwf	delay1, 0

Delay44usLoop

		decfsz	delay1, 1
		goto	Delay44usLoop
		return
delay5ms
		movlw	0xC2
		movwf	delay1,0
		movlw	0x0A
		movwf	delay2,0

Delay5msLoop
		decfsz	delay1, 1
		goto	d2
		decfsz	delay2, 1
d2		goto	Delay5msLoop
		return

    end