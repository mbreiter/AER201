#include <p18f4620.inc>

    errorlevel -302
    errorlevel -305

;global labels

    variable _waitknt=0

    udata 0xB0
regaddress res 1
databyte res 1
datachar res 1
tens_digit res 1
ones_digit res 1
convert_buffer res 1


    global write_rtc,read_rtc,rtc_convert,i2c_common_setup
    global regaddress, databyte, datachar, tens_digit, ones_digit, convert_buffer


;; I2C MACROS
;;
;; Sebastian K, commit 110219-2208
;; forked off PIC16 sample code
;; for PIC18F4620
;; relocatable labels



;I2C lowest layer macros;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

i2c_common_check_ack macro err_address ;If bad ACK bit received, goto err_address
    btfsc SSPCON2,ACKSTAT
    goto err_address
    endm

i2c_common_start macro
    ;input: none
    ;output: none
    ;desc: initiate start conditionon the bus
    bsf SSPCON2,SEN
    btfss PIR1, SSPIF
    bra $-2

    bcf PIR1, SSPIF ;clear SSPIF interrupt bit

    endm

i2c_common_stop macro
    ;input: none
    ;output: none
    ;desc: initiate stop condition on the bus
    bsf SSPCON2,PEN
    btfsc SSPCON2,PEN
    bra $-2
    bcf PIR1, SSPIF
    endm

i2c_common_repeatedstart macro
    ;input: none
    ;output: none
    ;desc: initiate repeated start on the bus. Usually used for
    ; changing direction of SDA without STOP event
    bsf SSPCON2,RSEN
    btfss PIR1, SSPIF
    bra $-2

    bcf PIR1, SSPIF ;clear SSPIF interrupt bit

    endm

i2c_common_ack macro
    ;input: none
    ;output: none
    ;desc: send an acknowledge to slave device
    bcf SSPCON2,ACKDT
    bsf SSPCON2,ACKEN
    btfsc SSPCON2,ACKEN
    bra $-2
    endm

i2c_common_nack macro
    ;input: none
    ;output: none
    ;desc: send an not acknowledge to slave device
    local   Again
    bsf SSPCON2,ACKDT
    bsf SSPCON2,ACKEN
Again
    btfsc SSPCON2,ACKEN
    goto Again
    endm

i2c_common_write macro

    ;input: W
    ;output: to slave device
    ;desc: writes W to SSPBUF and send to slave device. Make sure
    ; transmit is finished before continuing

    btfsc SSPSTAT, BF
    bra $-2

    movwf SSPBUF

    btfss PIR1, SSPIF
    bra $-2
    bcf PIR1, SSPIF ;clear SSPIF interrupt bit

    endm

i2c_common_read macro
    ;input: none
    ;output: W
    ;desc: reads data from slave and saves it in W.

    bsf SSPCON2,RCEN ;Begin receiving byte from
    btfss PIR1, SSPIF
    bra $-2
    bcf PIR1, SSPIF
    movf SSPBUF,w
    endm
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    code

i2c_common_setup
    ;input: none
    ;output: none
    ;desc: sets up I2C as master device with 100kHz baud rate

    movlw b'10000000'
    movwf SSPSTAT ;set I2C line leves, clear all flags.

    movlw 24 ;100kHz baud rate: 10000000 osc / [4*100000] -1
    movwf SSPADD ;RTC only supports 100kHz

    movlw b'00101000' ;Config SSP for Master Mode I2C
    movwf SSPCON1

    bsf SSPCON1,SSPEN ;Enable SSP module


    bcf PIR2, SSPIE ;don't cause an interrupt on SSPIF, please. We're just polling it.
    bcf PIR1, SSPIF
    bcf SSPSTAT, BF
    i2c_common_stop ;Ensure the bus is free
    return

    ;rtc Algorithms;;;;;;

write_rtc

    ;input: address of register in RTC
    ;output: none
    ;Desc: handles writing data to RTC
    ;Select the DS1307 on the bus, in WRITE mode
    i2c_common_start



    movlw 0xD0 ;DS1307 address | WRITE bit
    i2c_common_write
    i2c_common_check_ack WR_ERR1

    ;Write data to I2C bus (Register Address in RTC)
    movf regaddress,w ;Set register pointer in RTC
    i2c_common_write ;
    i2c_common_check_ack WR_ERR2

    ;Write data to I2C bus (Data to be placed in RTC register)
    movf databyte,w ;Write data to register in RTC
    i2c_common_write
    i2c_common_check_ack WR_ERR3

    goto WR_END
WR_ERR1
    nop
    movlw b'00001101'
    movwf LATA

    goto WR_END
WR_ERR2
    nop
    movlw b'00001111'
    movwf LATA
    bsf LATE,0
    goto WR_END
WR_ERR3
    nop
    movlw b'00001001'
    movwf LATA
    goto WR_END
WR_END
    i2c_common_stop ;Release the I2C bus
    return

    ;; ************************************************** READ RTC
read_rtc
    ;input: address of RTC
    ;output: DOUT or 0x75
    ;Desc: This reads from the selected address of the RTC
    ; and saves it into DOUT or address 0x75

    ;Select the DS1307 on the bus, in WRITE mode

    i2c_common_start

    movlw 0xD0 ;DS1307 address | WRITE bit
    i2c_common_write ;
    i2c_common_check_ack RD_ERR1

    ;Write data to I2C bus (Register Address in RTC)
    movf regaddress,w ;Set register pointer in RTC
    i2c_common_write ;
    i2c_common_check_ack RD_ERR2

    ;Re-Select the DS1307 on the bus, in READ mode
    i2c_common_repeatedstart
    movlw 0xD1 ;DS1307 address | READ bit
    i2c_common_write
    i2c_common_check_ack RD_ERR3

    ;Read data from I2C bus (Contents of Register in RTC)
    i2c_common_read
    movwf datachar
    i2c_common_nack ;Send acknowledgement of data reception


    goto RD_END

RD_ERR1
    nop
    movlw b'00000011'
    movwf LATA
    goto RD_END
RD_ERR2
    nop
    movlw b'00000101'
    movwf LATA
    goto RD_END
RD_ERR3
    nop
    movlw b'00000111'
    movwf LATA
    goto RD_END
    ;Release the I2C bus
RD_END
    i2c_common_stop
    return

rtc_convert
    ;input: W
    ;output: tens_digit, ones_digit
    ;desc: This subroutine converts the binary number
    ; in W into a two digit ASCII number and place
    ; each digit into the corresponding registers
    ; dig10 or dig1

    movwf convert_buffer ; B1 = HHHH LLLL
    swapf convert_buffer,w ; W = LLLL HHHH
    andlw 0x0f ; Mask upper four bits 0000 HHHH
    addlw 0x30 ; convert to ASCII
    movwf tens_digit ;saves into 10ths digit

    movf convert_buffer,w
    andlw 0x0f ; w = 0000 LLLL
    addlw 0x30 ; convert to ASCII
    movwf ones_digit ; saves into 1s digit
    return



    end