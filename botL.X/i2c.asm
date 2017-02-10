#include <p18f4620.inc>
#include <lcd.inc>
errorlevel -302
errorlevel -305
    CODE
 
I2C_temp    res		1
I2C_ACKDT   res		1
     
global I2C_Master_INIT, I2C_Master_WAIT,I2C_Master_START,I2C_Master_RSTART,I2C_Master_STOP,I2C_Master_WRITE, I2C_Master_READ,I2C_Master_ACK,CHECK_ACK, CHECK_NACK
global I2C_ACKDT
   
I2C_Condition	macro	 condition
    bsf	    SSPCON2, condition
    endm
   
CHECK_ACK
    btfsc   SSPCON2, ACKSTAT
    return

CHECK_NACK
    bsf	    SSPCON2, ACKDT
    bcf	    SSPCON2, ACKEN
TEST_NACK
    btfsc   SSPCON2, ACKEN
    goto    TEST_NACK
    return
    
I2C_Master_INIT
    clrf    SSPSTAT
    movlw   b'00101000'
    movwf   SSPCON1
    clrf    SSPCON2
    movlw   24		    ;100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf   SSPADD	    ;RTC only supports 100kHz
    bsf	    TRISC, 3
    bsf	    TRISC, 4
    return

I2C_Master_WAIT
WAIT    
    btfsc   SSPSTAT, 2
    goto    WAIT
   
    movlw   0x1F
    andwf   SSPCON2, w
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
    movf    SSPBUF, W
    return
    
I2C_Master_ACK
    bcf	    SSPCON2,ACKDT
    bsf	    SSPCON2,ACKEN
TEST_ACK
    btfsc   SSPCON2,ACKEN
    bra	    TEST_ACK
    return
    
    end
