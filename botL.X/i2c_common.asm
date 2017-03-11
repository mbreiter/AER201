#include <p18f4620.inc>
#include <lcd.inc>

errorlevel -302
errorlevel -305
errorlevel -205
errorlevel -203
errorlevel -207

variable _waitknt=0

udata_acs 0x60
regaddress res 1
databyte res 1
datachar res 1
tens_digit res 1
ones_digit res 1
convert_buffer res 1

global write_rtc,read_rtc,rtc_convert,i2c_common_setup, READ_COLOUR_I2C, WRITE_COLOUR_I2C, READ_ARDUINO, INIT_ARDUINO, INIT_RTC
global regaddress, databyte, datachar, tens_digit, ones_digit, convert_buffer
 
extern delay44us

i2c_common_check_ack macro err_address	;If bad ACK bit received, goto err_address
    btfsc   SSPCON2, ACKSTAT
    goto    err_address
endm

i2c_common_start macro 
    bsf	    SSPCON2, SEN
    btfss   SSPCON1, WCOL		; NOTE: I CHANGED THIS TO MAKE WORK, IDK Y THO
    bsf	    PIR1, SSPIF
    btfss   PIR1, SSPIF
    bra	    $-2    
    bcf	    PIR1, SSPIF			; clear SSPIF interrupt bit
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

    bcf	    PIR1, SSPIF			;clear SSPIF interrupt bit
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
    bcf	    PIR1, SSPIF		; clear SSPIF interrupt bit
endm

i2c_common_read macro
    bsf	    SSPCON2,RCEN	; begin receiving byte
    btfss   PIR1, SSPIF
    bra	    $-2
    bcf	    PIR1, SSPIF
    movff   SSPBUF, WREG
endm

code

i2c_common_setup
    movlw   b'00000000'
    movwf   SSPSTAT		; set I2C line leves, clear all flags.

    movlw   b'00101000'		; config SSP for Master Mode I2C
    movwf   SSPCON1
    
    movlw   b'00000000'
    movwf   SSPCON2
    
    movlw   d'24'		; 100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf   SSPADD		; rtc only supports 100kHz

    bsf	    SSPCON1, SSPEN	; enable SSP module

    bcf	    PIR2, SSPIE
    bcf	    PIR1, SSPIF
    bcf	    SSPSTAT, BF
    i2c_common_stop		; ensure the bus is free
return

write_rtc
    i2c_common_start

    movlw   0xd0		; ds1307 address | WRITE bit
    i2c_common_write

    ; write data to I2C bus
    movff   regaddress, WREG	; set register pointer in RTC
    i2c_common_write

    ; write data to I2C bus
    movff   databyte, WREG	; write data to register in RTC
    i2c_common_write

    i2c_common_stop		; release the I2C bus
return

read_rtc
    i2c_common_start
    
    movlw   0xd0		; ds1307 address | WRITE bit
    i2c_common_write
        
    ; write data to I2C bus
    movff   regaddress, WREG	; set register pointer in RTC
    i2c_common_write
        
    ; re-select the ds1307 on the bus, in READ mode
    i2c_common_repeatedstart
    movlw   0xd1		; ds1307 address | READ bit
    i2c_common_write
    
    ; read data from I2C bus
    i2c_common_read
    movwf   datachar
    i2c_common_nack		; send acknowledgement of data reception
    
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

INIT_RTC
    i2c_common_start
    
    movlw   0xd0
    i2c_common_write
    
    movlw   0x0e
    i2c_common_write
    
    movlw   0x04
    i2c_common_write
    
    i2c_common_stop 
return
    
READ_COLOUR_I2C  
    i2c_common_start
    
    movlw   0x52		; TCS34725 address 0x29 << 1, WRITE bit (+0)
    i2c_common_write  
    
    ; write command to I2C bus (Register Address in TCS34725)
    movff   regaddress, WREG
    iorlw   0x80		; command bit
    i2c_common_write
    
    ; re-select the tcs34725 on the bus, in READ mode
    i2c_common_repeatedstart
    movlw   0x53		; tcs34725 address 0x29 << 1, READ bit (+1)
    i2c_common_write
    
    ; read data from I2C bus (Contents of Register in TCS34725)
    i2c_common_read
    movwf   databyte
    i2c_common_nack		; send acknowledgement of data receipt
    
    i2c_common_stop
return	
	
WRITE_COLOUR_I2C
    i2c_common_start

    movlw   0x52		; TCS34725 address | WRITE bit
    i2c_common_write

    ; write data to I2C bus (Register Address in TCS34725)
    movff   regaddress, WREG
    iorlw   0x80		; command bits
    i2c_common_write

    ; write data to I2C bus (Data to be placed in TCS34725 register)
    movff   databyte, WREG
    andlw   0xff
    i2c_common_write

    i2c_common_stop		; Release the I2C bus
	
return

INIT_ARDUINO
    i2c_common_start
    
    movlw   0x10		; Arudino address << 1 + WRITE
    i2c_common_write
    
    i2c_common_stop
return
	
	
READ_ARDUINO
    i2c_common_start

    i2c_common_repeatedstart
    movlw   0x11		; arudino address << 1 + READ
    i2c_common_write
    
    i2c_common_read
    i2c_common_nack
    
    i2c_common_stop
return

    end