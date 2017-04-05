    extern  i2c_common_setup, regaddress, databyte, datachar, READ_ARDUINO, INIT_ARDUINO, WR_DATA, delay5ms
    
    ARDUINO_READ macro bottle_sort
	call	READ_ARDUINO
	movff	WREG, bottle_sort
    endm
    
    
    



