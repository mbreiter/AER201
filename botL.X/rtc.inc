extern write_rtc,read_rtc,rtc_convert,i2c_common_setup, regaddress, databyte, datachar,tens_digit,ones_digit

variable _loopknt=0

rtc_resetAll macro

    clrf    databyte
    movlw   6
    movwf   regaddress ;start at 6 and go down, and set all to 0

    del_loop#v(_loopknt)
    call    write_rtc
    decfsz  regaddress
    bra	    del_loop#v(_loopknt)
    call    write_rtc ;sets registers 0-6 to zero
    _loopknt set _loopknt+1
endm

rtc_set macro addliteral,datliteral
    movlw   addliteral
    movwf   regaddress
    movlw   datliteral
    movwf   databyte
    call    write_rtc
endm

rtc_read macro addliteral
    movlw   addliteral
    movwf   regaddress
    call    read_rtc
    movf    datachar,w
    call    rtc_convert
endm