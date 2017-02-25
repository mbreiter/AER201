#include <p18f4620.inc>
#include <lcd.inc>

errorlevel -302
errorlevel -305
errorlevel -205
errorlevel -203
errorlevel -207
    
;global labels

variable _waitknt=0

udata
regaddress res 1
databyte res 1
datachar res 1
data_colourL res 1
data_colourH res 1
tens_digit res 1
ones_digit res 1
convert_buffer res 1

global write_rtc,read_rtc,rtc_convert,i2c_common_setup, READ_COLOUR_I2C, WRITE_COLOUR_I2C, READ_ARDUINO, INIT_ARDUINO
global regaddress, databyte, datachar, tens_digit, ones_digit, convert_buffer
global data_colourL, data_colourH

i2c_common_check_ack macro err_address ;If bad ACK bit received, goto err_address
    btfsc   SSPCON2, ACKSTAT
    goto    err_address
endm

i2c_common_start macro 
    bsf	    SSPCON2, SEN
    btfss   SSPCON1, WCOL	; NOTE: I CHANGED THIS TO MAKE WORK, IDK Y THO
    bsf	    PIR1, SSPIF
    btfss   PIR1, SSPIF
    bra	    $-2    
    bcf	    PIR1, SSPIF		; clear SSPIF interrupt bit
endm

i2c_common_stop macro
    bsf	    SSPCON2,PEN
    btfsc   SSPCON2,PEN
    bra	    $-2
    bcf	    PIR1, SSPIF
endm

i2c_common_repeatedstart macro
    bsf	    SSPCON2,RSEN
    btfss   PIR1, SSPIF
    bra	    $-2

    bcf	    PIR1, SSPIF ;clear SSPIF interrupt bit
endm

i2c_common_ack macro
    bcf	    SSPCON2,ACKDT
    bsf	    SSPCON2,ACKEN
    btfsc   SSPCON2,ACKEN
    bra	    $-2
endm

i2c_common_nack macro
    bsf	    SSPCON2,ACKDT
    bsf	    SSPCON2,ACKEN

    _waitl#v(_waitknt)
    btfsc   SSPCON2,ACKEN
    goto    _waitl#v(_waitknt)
    _waitknt	set _waitknt+1
endm

i2c_common_write macro 
    btfsc   SSPSTAT, BF
    bra	    $-2
    movwf   SSPBUF
    btfss   PIR1, SSPIF
    bra	    $-2
    bcf	    PIR1, SSPIF ;clear SSPIF interrupt bit
endm

i2c_common_read macro
    bsf	    SSPCON2,RCEN ;Begin receiving byte from
    btfss   PIR1, SSPIF
    bra	    $-2
    bcf	    PIR1, SSPIF
    movff   SSPBUF, WREG
endm

code

i2c_common_setup
    movlw   b'00000000'
    movwf   SSPSTAT ;set I2C line leves, clear all flags.

    movlw   b'00101000' ;Config SSP for Master Mode I2C
    movwf   SSPCON1
    
    movlw   b'00000000'
    movwf   SSPCON2
    
    movlw   24 ;100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf   SSPADD ;RTC only supports 100kHz

    bsf	    SSPCON1,SSPEN ;Enable SSP module

    bcf	    PIR2, SSPIE
    bcf	    PIR1, SSPIF
    bcf	    SSPSTAT, BF
    i2c_common_stop ;Ensure the bus is free
return

write_rtc
    i2c_common_start

    movlw   0xD0 ;DS1307 address | WRITE bit
    i2c_common_write
    i2c_common_check_ack WR_ERR1_RTC

    ;Write data to I2C bus (Register Address in RTC)
    movff   regaddress, WREG ;Set register pointer in RTC
    i2c_common_write ;
    i2c_common_check_ack WR_ERR2_RTC

    ;Write data to I2C bus (Data to be placed in RTC register)
    movff   databyte, WREG ;Write data to register in RTC
    i2c_common_write
    i2c_common_check_ack WR_ERR3_RTC

    goto    WR_END_RTC

    WR_ERR1_RTC
	nop
	movlw	b'00001101'
	movwf	LATA	
	goto	WR_END
    
    WR_ERR2_RTC
	nop
	movlw   b'00001111'
	movwf   LATA
	bsf	LATE,0
	goto    WR_END
	
    WR_ERR3_RTC
	nop
	movlw	b'00001001'
	movwf	LATA
	goto	WR_END
    WR_END_RTC
	i2c_common_stop ;Release the I2C bus
return

read_rtc
    i2c_common_start

    movlw   0xD0 ;DS1307 address | WRITE bit
    i2c_common_write ;
    i2c_common_check_ack RD_ERR1_RTC
    
    ;Write data to I2C bus (Register Address in RTC)
    movff   regaddress, WREG	; Set register pointer in RTC
    i2c_common_write
    i2c_common_check_ack RD_ERR2_RTC

    ;Re-Select the DS1307 on the bus, in READ mode
    i2c_common_repeatedstart
    movlw   0xD1		; DS1307 address | READ bit
    i2c_common_write
    i2c_common_check_ack RD_ERR3_RTC

    ;Read data from I2C bus (Contents of Register in RTC)
    i2c_common_read
    movwf   datachar
    i2c_common_nack ;Send acknowledgement of data reception

    goto RD_END_RTC

    RD_ERR1_RTC
	nop
	movlw	b'00000011'
	movwf	LATA
	goto	RD_END
    RD_ERR2_RTC
	nop
	movlw	b'00000101'
	movwf	LATA
	goto	RD_END
    RD_ERR3_RTC
	nop
	movlw	b'00000111'
	movwf	LATA
	goto	RD_END
    RD_END_RTC
	i2c_common_stop
return

rtc_convert
    movwf   convert_buffer	; B1 = HHHH LLLL
    swapf   convert_buffer, WREG; W = LLLL HHHH
    andlw   0x0f		; Mask upper four bits 0000 HHHH
    addlw   0x30		; convert to ASCII
    movwf   tens_digit		; saves into 10ths digit

    movff   convert_buffer, WREG
    andlw   0x0f		; w = 0000 LLLL
    addlw   0x30		; convert to ASCII
    movwf   ones_digit		; saves into 1s digit
return
    
READ_COLOUR_I2C  
    i2c_common_start
    
    movlw   0x52		; TCS34725 address 0x29 << 1, WRITE bit (+0)
    i2c_common_write  
    i2c_common_check_ack RD_ERR1
    
    ;Write command to I2C bus (Register Address in TCS34725)
    movff   regaddress, WREG
    iorlw   0x80		; command bit
    
    i2c_common_write
    i2c_common_check_ack RD_ERR2
    
    ;Re-Select the TCS34725 on the bus, in READ mode
    i2c_common_repeatedstart
    movlw   0x53		; TCS34725 address 0x29 << 1, READ bit (+1)
    i2c_common_write
    i2c_common_check_ack RD_ERR3
    
    ;Read data from I2C bus (Contents of Register in TCS34725)
    i2c_common_read
    movwf   databyte
    i2c_common_nack		; Send acknowledgement of data reception
    
    goto RD_END

    RD_ERR1
	nop
	movlw	b'00000011'
	movwf	LATA
	goto	RD_END
    RD_ERR2
	nop
	movlw	b'00000101'
	movwf	LATA
	goto	RD_END
    RD_ERR3
	nop
	movlw	b'00000111'
	movwf	LATA
	goto	RD_END
    RD_END
	i2c_common_stop
return
		
I2C_Condition	macro	 condition
    bsf	    SSPCON2, condition
    endm
    
I2C_Master_INIT
    clrf    SSPSTAT
    movlw   b'00101000'
    movwf   SSPCON1
    clrf    SSPCON2
    movlw   24		    ; 100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf   SSPADD	    ; RTC only supports 100kHz
    bsf	    TRISC, 3
    bsf	    TRISC, 4
    return

I2C_Master_WAIT
WAIT    
    btfsc   SSPSTAT, 2
    goto    WAIT
   
    movlw   0x1f
    andwf   SSPCON2, 0
    iorlw   0x00
    bnz	    WAIT
    return
        
I2C_Master_START
    call    I2C_Master_WAIT
    I2C_Condition   SEN
    return

I2C_Master_RSTART
    call    I2C_Master_WAIT
    I2C_Condition   RSEN
    return
    
I2C_Master_STOP
    call    I2C_Master_WAIT
    
    I2C_Condition   PEN
    return
    
I2C_Master_WRITE 
    call    I2C_Master_WAIT
    movwf   SSPBUF
HOLD_UP
    btfsc   SSPSTAT, 2
    goto    HOLD_UP
    return   
    
I2C_Master_READ
    bsf	    SSPCON2, RCEN
    call    I2C_Master_WAIT   
LOOP
    btfsc   SSPCON2, RCEN
    bra LOOP
    movff    SSPBUF, W
    return
    
I2C_Master_ACK
    bcf	    SSPCON2,ACKDT
    bsf	    SSPCON2,ACKEN
TEST_ACK
    btfsc   SSPCON2,ACKEN
    bra	    TEST_ACK
    return	
	
WRITE_COLOUR_I2C
    i2c_common_start

    movlw   0x52		; TCS34725 address | WRITE bit
    i2c_common_write
    i2c_common_check_ack WR_ERR1

    ;Write data to I2C bus (Register Address in TCS34725)
    movff   regaddress, WREG
    iorlw   0x80		; command bits
    i2c_common_write
    i2c_common_check_ack WR_ERR2

    ;Write data to I2C bus (Data to be placed in TCS34725 register)
    movff   databyte, WREG
    andlw   0xff
    i2c_common_write
    i2c_common_check_ack WR_ERR3

    goto    WR_END

    WR_ERR1
	nop
	movlw	b'00001101'
	movwf	LATA	
	goto	WR_END
    
    WR_ERR2
	nop
	movlw   b'00001111'
	movwf   LATA
	bsf	LATE,0
	goto    WR_END
	
    WR_ERR3
	nop
	movlw	b'00001001'
	movwf	LATA
	goto	WR_END
    WR_END
	i2c_common_stop		; Release the I2C bus
	
return

INIT_ARDUINO
    i2c_common_start
    
    movlw   0x10		; Arudino address << 1 + WRITE
    i2c_common_write
    i2c_common_check_ack WR_ERR1

    i2c_common_stop
return
	
	
READ_ARDUINO
    i2c_common_start
    
    movlw   0x11		; Arudino address << 1 + READ
    i2c_common_write
    i2c_common_check_ack WR_ERR1
    
    i2c_common_read
;    movwf   databyte		; put result into databyte
    i2c_common_nack
    
    i2c_common_stop
    
return

    end