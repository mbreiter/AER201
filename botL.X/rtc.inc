;External labels
    extern write_rtc,read_rtc,rtc_convert,i2c_common_setup, regaddress, databyte, datachar

;; RTC MACROS
;;
;; Sebastian K, commit 110219-2205
;; forked off PIC16 sample code
;; for PIC18F4620
;; relocatable labels, declared in another file

    variable _loopknt=0

rtc_resetAll macro
;input: none
;output: none
;desc: Resets all the time keeping registers on the RTC to zero

    local      Again

    clrf databyte
    movlw 6
    movwf regaddress ;start at 6 and go down, and set all to 0

Again
    call write_rtc
    decfsz regaddress
    bra Again
    call write_rtc ;sets registers 0-6 to zero
    endm




rtc_set macro addliteral,datliteral
;input: addliteral: value of address
; datliteral: value of data
;output: none
;desc: loads the data in datliteral into the
; address specified by addliteral in the RTC

    movlw addliteral
    movwf regaddress
    movlw datliteral
    movwf databyte
    call write_rtc
    endm

rtc_read macro addliteral
;input: addliteral
;output: datachar, tens_digit, ones_digit
;desc: From the selected register in the RTC, read the data
; and load it into 0x75. 0x75 is also converted into
; ASCII characters and the tens digit is placed into
; 0x77 and the ones digit is placed in 0x78
    movlw addliteral
    movwf regaddress
    call read_rtc
    movf datachar,w
    call rtc_convert
    endm