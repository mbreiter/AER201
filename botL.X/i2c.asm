#include <p18f4620.inc>
#include <lcd.inc>
errorlevel -302
errorlevel -305
    CODE
 
I2C_temp    res		1
I2C_ACKDT   res		1
    
global I2C_Master_INIT, I2C_Master_WAIT,I2C_Master_START,I2C_Master_RSTART,I2C_Master_STOP,I2C_Master_WRITE, I2C_Master_READ 
global I2C_ACKDT
    
    
I2C_Master_INIT
    movlw   b'00000000'
    movwf   SSPSTAT
    movlw   b'00101000'
    movwf   SSPCON1
    movlw   b'00000000'
    movwf   SSPCON2
    movlw   24		    ;100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf   SSPADD	    ;RTC only supports 100kHz
    bsf	    TRISC, 3
    bsf	    TRISC, 4
    return
    
I2C_Master_WAIT
WAIT
    movf I2C_temp, ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F))
    bnz    WAIT
    return
        
I2C_Master_START
    call    I2C_Master_WAIT
    bsf	    SSPCON2, SEN
    return

I2C_Master_RSTART
    call    I2C_Master_WAIT
    bsf	    SSPCON2, RSEN
    return
    
I2C_Master_STOP
    call    I2C_Master_WAIT
    bsf	    SSPCON2, PEN
    return
    
I2C_Master_WRITE
    call    I2C_Master_WAIT
    movf    SSPBUF, W
    return   
    
I2C_Master_READ
    call    I2C_Master_WAIT
    bsf	    SSPCON2, RCEN
    call    I2C_Master_WAIT
    movf    SSPBUF
    call    I2C_Master_WAIT
    btfsc   I2C_ACKDT, 1
    bsf	    SSPCON2, ACKDT
    bsf	    SSPCON2, ACKEN
    return
    
    end
