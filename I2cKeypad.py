# -*- coding: utf-8 -*-

'''
    I2cKeypad.py - class library for using I2C Keypad Backpack

    Written by: Gary Muhonen  gary@dcity.org

    Versions:
        1.0.0 - 9/1/2018
            Original release.

    Short Description:

        These files provide a software library and demo program for the Raspberry Pi.

        The library files provide useful functions to make it easy
        to communicate with matrix keypads (like a 4x4 keypad)
        that use the I2C communication protocol. The demo
        program shows the usage of the functions in the library.

        The keypad must connect to the I2C bus using a MCP23008 8 bit interface chip.
        A backback board with the MCP23008 chip is available and details are in the link below.


    https://www.dcity.org/portfolio/i2c-keypad-library/
    This link has details including:
        * software library installation for use with Arduino, Particle and Raspberry Pi boards
        * list of functions available in these libraries
        * a demo program (which shows the usage of most library functions)
        * info on keypads that work with this software
        * hardware design for a backpack board for keypads, available on github
        * info on backpack “bare” pc boards available from OSH Park.

    License Information:  https://www.dcity.org/license-information/

    Notes:
        1. You must enable I2C on your Raspberry Pi board (see your particular operating system documentation).
            On Raspian: Menu...Preferences...Raspberry Pi Configuration...Interfaces...Enable I2C
        2. This software was tested on a RASPBERRY PI 3 MODEL B, running Rasbian and Python 3.5.2
        3. The I2cKeypad class uses a software timer from the python "threading" class. If you are not using
            a hardware interrupt from the keypad, then this timer will cause the software to check for a keypress
            every 10 milliseconds.
        4. The user provides a list of lists (__keyMap) that contains ASCII characters (like 'A' or '0').
            When there is a valid keypress, that character and put it into a bytearray (__keyBuffer), which must be
            an unsigned byte value in range of 0-255. The ord() function is used to convert the ASCII chars to the byte value.
            When the user calls a function like getKey(), these byte values are converted back to ASCII characters using the chr() function.
        5. If you are using a hardware interrupt line from the keypad, then this software requires that you have GPIO library version
            0.5.2a or above. See the GPIO import line below.


'''




import smbus                            # import the i2c library
from time import sleep                  # import the sleep functions, for time delays
import time                             # used to get the current time in milliseconds
import threading                        # used to get timer interrupts

import RPi.GPIO as GPIO                 # used for using hardware interrupt from the keypad, using one of the GPIO pins
print ("GPIO verion 0.5.2a or higher is required. You are using version: " + GPIO.VERSION)      # code requires version 0.5.2a or above

# python SMBus commands: http://www.raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2
i2c = smbus.SMBus(1)                    # create an i2c object for writing/reading from i2c



# create a class for the i2c keypad
class I2cKeypad():

    # Constants used in this library code.

    # registers in MCP23008 chip
    MCP_IODIR     = 0x00    # I/O direction register 1=input, 0=output (power on = 0xff)
    MCP_IPOL      = 0x01    # Input polarity register 1=input reversed, 0=input normal (0x00)
    MCP_GPINTEN   = 0x02    # Interrupt on change control register 1=gpio pin enabled for interrupt 0=no interrupt for that pin (0x00)
    MCP_DEFVAL    = 0x03    # Default Compare register for interrupt change (0x00)
    MCP_INTCON    = 0x04    # Interrupt control register 1=pin is compared against DEFVAL register, 0=pin is compared against previous pin value (0x00)
    MCP_IOCON     = 0x05    # Configuration register (0x00)
    MCP_GPPU      = 0x06    # Pullup resistor configuration register 1=pullup enabled, 0=pullup disabled
    MCP_INTF      = 0x07    # Interrupt flag register (read only) 1=an interrupt condition on that pin, 0=no interrupt (0x00)
    MCP_INTCAP    = 0x08    # Interrupt capture register contains the contents of GPIO port at time of interrupt (0x00)
    MCP_GPIO      = 0x09    # GPIO register is the input value of the port (writing writes to the OLAT register)
    MCP_OLAT      = 0x0A    # Output latch register (0x00)

    '''
    IOCON bits
    bit 7   0   Unimplemented: Read as ‘0’.
    bit 6   0   Unimplemented: Read as ‘0’.
    bit 5   1   SEQOP: Sequential Operation mode bit. 1 = Sequential operation disabled, address pointer does not increment. 0 = Sequential operation enabled, address pointer increments.
    bit 4   0   DISSLW: Slew Rate control bit for SDA output. 1= Slewratedisabled. 0= Slewrateenabled.
    bit 3   0   HAEN: Hardware Address Enable bit (MCP23S08 only). Address pins are always enabled on MCP23008. 1 = Enables the MCP23S08 address pins. 0 = Disables the MCP23S08 address pins.
    bit 2   1   ODR: This bit configures the INT pin as an open-drain output. 1 = Open-drain output (overrides the INTPOL bit). 0 = Active driver output (INTPOL bit sets the polarity).
    bit 1   0   INTPOL: This bit sets the polarity of the INT output pin. 1= Active-high. 0= Active-low.
    bit 0   0   Unimplemented: Read as ‘0’.

    In our code we set the IOCON register to 0x24
    '''

    MCP_IOCON_VALUE = 0x24         # initial value for the MCP IOCON register. SEQOP = 1: sequential operation disabled, ODR=1: Int pin is open drain

    # __scanKeys() states - there is a state machine in this function, and these are the state possibilities
    WAITING_FOR_NEW_KEY_PRESS = 0
    #WAITING_DEBOUNCE_TIME = 1               # not currently used in this RPi version of the code
    WAITING_FOR_NO_KEYS_PRESSED = 2

    # values returned by various INTERNAL functions (these are NOT useful for user's code)
    NO_KEYS_PRESSED = -1
    TIMEOUT_PERIOD_EXCEEDED = -2
    MAX_NUM_CHARACTERS_REACHED = -3
    TERMINATOR_CHAR_RECEIVED = -4
    MULTIPLE_KEYS_PRESSED = -5

    # interrupt Constants
    KEYPAD_USING_INTERRUPTS = 1
    KEYPAD_NOT_USING_INTERRUPTS = 0

    KEYPAD_BUFFER_SIZE = 32                 # a bytearray is used to store incoming keypad characters, 32 bytes long.
    RETURN_NO_KEY_IN_BUFFER = chr(0)        # value returned by getKey(), peekKey(), getKeyUntil() when there are no keys in the buffer to be returned

    TIMER_PERIOD = 10                       # If not using hardware interrupts, a threading timer is used to repeated call __scanKeys()
                                            #   every 10 millisecods. If you make this too big, then you could miss keypresses.

    # constructor to create I2cKeypad object
    def __init__(self, keyMap, rowPins, colPins, i2cAddress, usingKeypadInterrupts, keypadInterruptPin):

        # private vars used by functions in the class
        self.__keyMap = keyMap[:]               # list of lists for mapping which ascii character is returned for each key on the keypad
        self.__rowPins = rowPins[:]             # list for mapping which pin on the MCP23008 corresponds to each row on the keypad
        self.__colPins = colPins[:]             # list for mapping which pin on the MCP23008 corresponds to each column on the keypad

        self.__i2cAddress = i2cAddress          # the i2c address of the mcp23008 chip
        self.__usingKeypadInterrupts = usingKeypadInterrupts   # true if the user is using hardware interrupts from the keypad
        self.__keypadInterruptPin = keypadInterruptPin          # hardware BCM pin number used for the keypd interrupt line.

        self.__keyBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])    # 32 element bytearray for storing the keys that are pressed, first in is at position 0
        self.__keyBufferHead = 0            # keep track of the next key to put into the buffer
        self.__keyBufferTail = 0            # keep track of the next key to take out of the buffer


        # create a mask byte where each row pin will have a high bit in _inputPinsMask (keypad rows are inputs on the MCP23008 chip)
        self.__inputPinsMask = 0
        for pin in self.__rowPins:
            self.__inputPinsMask = self.__bitSet(self.__inputPinsMask, self.__rowPins[pin])     # set the bit for each pin that is in the row list


        # set up registers in the MCP230008 chip
        self.__mcpWriteByte(I2cKeypad.MCP_IODIR, self.__inputPinsMask)   # write 1's for the bits that are inputs
        self.__mcpWriteByte(I2cKeypad.MCP_IPOL, 0)                 # don't invert the polarity of any input pins
        self.__mcpWriteByte(I2cKeypad.MCP_GPINTEN, 0)              # no interrupt pins are enabled initially... will be set to all ones when interrupts are enabled
        self.__mcpWriteByte(I2cKeypad.MCP_DEFVAL, 0xff)               # if we use interrupts, then this needs to be all ones to detect key pushed
        self.__mcpWriteByte(I2cKeypad.MCP_INTCON, 0xff)               # interrupt control register... if we use interrupts, then this needs to be all ones to detect key pushed
        self.__mcpWriteByte(I2cKeypad.MCP_IOCON, I2cKeypad.MCP_IOCON_VALUE)  # configuration register.
        self.__mcpWriteByte(I2cKeypad.MCP_GPPU, 0xff)              # set all input pins to have pullup resistor
        # self.__mcpWriteByte(I2cKeypad.MCP_INTF, 0)              # Interrupt Flag is read only... no need to initialize it
        # self.__mcpWriteByte(I2cKeypad.MCP_INTCAP, 0)            # Interrupt Capture is read only... no need to initialize it
        # self.__mcpWriteByte(I2cKeypad.MCP_GPIO, 0)              # GPIO is read only... no need to initialize it
        self.__mcpWriteByte(I2cKeypad.MCP_OLAT, 0)                 # Output Latch: set all the output pins low initially

        self.__keypadState = I2cKeypad.WAITING_FOR_NEW_KEY_PRESS  # setup our state variable used in __scanKeys()

        if(self.__usingKeypadInterrupts == I2cKeypad.KEYPAD_NOT_USING_INTERRUPTS):     # if NOT using hardware interrupts, set up timer to call __scanKeys after debounce time us up
            keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)
            keypadTimer.start()
        else:                       # else we are using hardware interrupts
            # This block of code is run if you are using a hardware interrupt line from the keypad, to trigger an interrupt when a key is pressed
            # inform the GPIO module that we are using the chip's numbering method
            GPIO.setmode(GPIO.BCM)
            # set pin keypadInterruptPin as an input, that we will use as the keypad interrupt line. It goes low when a key is pressed on the keypad
            GPIO.setup(keypadInterruptPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # set pin 23 to be an input with a pullup
            GPIO.add_event_detect(self.__keypadInterruptPin,GPIO.FALLING)                             # tell GPIO to look for a falling input condition
            GPIO.add_event_callback(self.__keypadInterruptPin,self.__interruptHandler)                # call __interruptHandler() when a key is pressed on the keypad
            self.__mcpEnableInterrupts()                        # enable hardware interrupts from the keypad (mcp chip)


    # Public functions in this library that the user can call

    # return the current time in milliseconds
    # parameters
    #    none
    # returns
    #    an integer containing the current time in milliseconds... typcially used for timing events, or waiting for a time to pass.
    def millis(self):
        return int(round(time.time() * 1000))


    # return the number of keys in the keypad buffer
    # parameters
    #    none
    # returns
    #    0 if there is no keys in the buffer
    #    else returns the number of keys available to read from the buffer
    def getKeyCount(self):
        return (I2cKeypad.KEYPAD_BUFFER_SIZE + self.__keyBufferHead - self.__keyBufferTail) % I2cKeypad.KEYPAD_BUFFER_SIZE


    # peek at the first key in the keypad buffer, but don't remove it from the buffer (you can still run getKey() to retreive it)
    # parameters
    #    none
    # returns
    #    RETURN_NO_KEY_IN_BUFFER - chr(0) -  if the keypad buffer is empty (no keys pressed)
    #    the ASCII value of the next key (from the _keyMap list)
    def peekKey(self):
        if (self.__keyBufferHead == self.__keyBufferTail):       # if no key in buffer return chr(0)
            return I2cKeypad.RETURN_NO_KEY_IN_BUFFER
        else:                                                    # else return the ASCII character of the next available keypress in the buffer
            return chr(self.__keyBuffer[self.__keyBufferTail])


    # get the next key from the keypad buffer
    # parameters
    #    none
    # returns
    #     RETURN_NO_KEY_IN_BUFFER - chr(0) - if there is no key in the buffer
    #     the ASCII value of the next key pressed (from the _keyMap list)
    def getKey(self):
        if (self.__keyBufferHead == self.__keyBufferTail):      # if no key in buffer return chr(0)
            return I2cKeypad.RETURN_NO_KEY_IN_BUFFER
        else:
            key = chr(self.__keyBuffer[self.__keyBufferTail])       # return a character by using the chr() fcn.
            self.__keyBufferTail = (self.__keyBufferTail + 1) % I2cKeypad.KEYPAD_BUFFER_SIZE
            return key             # return the ASCII character for the key press



    # flush out the _keyBuffer  (removes all previous unread keys from the keypad buffer)
    # parameters
    #    none
    # returns
    #    nothing
    def flushKeys(self):
        self.__keyBufferTail = self.__keyBufferHead




    # get one keypad character or wait UNTIL one key is pressed OR we timeout
    # parameters
    #    timeoutPeriod - amount of time we will wait for one keypad character in milliseconds. If 0, then we
    #                      will wait forever for a key press
    # returns
    #    the ASCII value of the key pressed (from the _keyMap list)
    #    RETURN_NO_KEY_IN_BUFFER if terminated by timeout (no key in buffer)
    def getKeyUntil(self, timeoutPeriod):
        currentTime = 0    # save the current time
        key = 0                   # key read from keypad

        currentTime = self.millis()       # get the current time so we can check for a timeout condition
        # keep waiting for more keypad presses, until a termination contidion occurs
        while(1):
            # check if the timeout feature is enabled (i.e. timeoutPeriod > 0) and if we have timed out
            if (timeoutPeriod and (self.millis() - currentTime) >= timeoutPeriod):
                return I2cKeypad.RETURN_NO_KEY_IN_BUFFER                       # return RETURN_NO_KEY_IN_BUFFER of we timed out before key was pressed.
            key = self.getKey() # read a key from the keypad buffer, this function also calls __scanKeys() to check for more keypresses
            # if there aren't any chars in the keypad buffer, then just continue to try again
            if (key == I2cKeypad.RETURN_NO_KEY_IN_BUFFER):
                continue                 # no keys to read
            else:
                return key   # return the ASCII keypad value


    # private functions ********************************
    # These functions below you typically would not call from your main program, as they are intended to be used by other internal class functions


    # interrupt handler for when a keyboard interrupt occurs.
    def __interruptHandler(self,channel):
        self.__scanKeys()


    # Scans the keypad for valid keys, and puts valid keys into _keyBuffer[] (which is a private buffer used by this library)
    # This function is called by most of the functions in this library to check if there are any keys being pressed.
    # The user should call this function (or one of the functions that calls this function) often (every 10ms or less), so that keys are not missed.
    # This function should be run often in you main loop, unless you are using some timer feature that calls this function often (every 10ms is a good period).
    # parameters
    #    none
    # returns
    #    nothing
    # new key presses are added to the _keyBuffer[] list.
    def __scanKeys(self):
        # an interrupt occurred... disable hardware interrupts until it's ok for another hardware interrupt
        if(self.__usingKeypadInterrupts == I2cKeypad.KEYPAD_USING_INTERRUPTS):
            self.__mcpDisableInterrupts()

        # this is a state machine, where we jump to a different case depending on what we are waiting for to happen

        # check to see if we are waiting keypress to occur
        if (self.__keypadState == I2cKeypad.WAITING_FOR_NEW_KEY_PRESS):   # this state is waiting for a new keypress to occur

            key = self.__checkForKeyPress()
            #key = self.__checkForKeyPress()
            # check if no keys are pressed, if so, just re-enable either hardware interrupt or a timer interrupt
            if (key == I2cKeypad.NO_KEYS_PRESSED):
                if(self.__usingKeypadInterrupts == I2cKeypad.KEYPAD_NOT_USING_INTERRUPTS):                   # if NOT using interrupts, set up timer to call __scanKeys after debounce time us up
                    keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)
                    keypadTimer.start()
                else:
                    self.__mcpEnableInterrupts()          # if using keypad interrupts, re enable them and wait for a hardware interrupt.

            # check if multiple keys are pressed (which we don't allow... it is considered the same as no keys pressed)
            elif (key == I2cKeypad.MULTIPLE_KEYS_PRESSED):
                self.__keypadState = I2cKeypad.WAITING_FOR_NO_KEYS_PRESSED   # go to waiting for no keys to be pressed, since we have to many keys pressed
                keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)    # wait debounce time and come back to this function
                keypadTimer.start()


            # else we have a valid key pressed, and only 1 key pressed
            else:

                # save the key to keyBuffer
                self.__keyBuffer[self.__keyBufferHead] = key                   # save key in self.__keyBuffer[]
                self.__keyBufferHead = (self.__keyBufferHead + 1) % I2cKeypad.KEYPAD_BUFFER_SIZE   # increment the index in the buffer array

                self.__keypadState = I2cKeypad.WAITING_FOR_NO_KEYS_PRESSED   # go to waiting for no keys to be pressed, so we don't get dup keys
                keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)    # wait debounce time and come back to this function
                keypadTimer.start()

                # Note: not using a debounce period with the Raspberry Pi... The RPi is a little slow, so we assume the keypad
                #       is done bouncing by the time this code has begun to run.
                '''
                # check if we are waiting for a debounce time period to expire
                elif (self.__keypadState == I2cKeypad.WAITING_DEBOUNCE_TIME):   # this state is used after a key was pressed and we are waiting for the debounce time to have passed.

                    key = self.__checkForKeyPress()
                    #print(key)
                    # check if we still have a key pressed after this debounce time
                    # check if multiple keys are pressed
                    if (key == I2cKeypad.MULTIPLE_KEYS_PRESSED):
                        self.__keypadState = I2cKeypad.WAITING_FOR_NO_KEYS_PRESSED   # go to waiting for no keys to be pressed, since we have to many keys pressed

                    # check if no keys are pressed
                    elif (key == I2cKeypad.NO_KEYS_PRESSED):
                        self.__keypadState = I2cKeypad.WAITING_FOR_NEW_KEY_PRESS   # go to waiting for new keys to be pressed, since we have no keys pressed

                    else:     #(key >= 0):          #else we have a valid key from the keypad
                        #print("yyy")
                        self.__keyBuffer[self.__keyBufferHead] = key                   # save key in self.__keyBuffer[]
                        self.__keyBufferHead = (self.__keyBufferHead + 1) % I2cKeypad.KEYPAD_BUFFER_SIZE   # increment the index in the buffer array
                        self.__keypadState = I2cKeypad.WAITING_FOR_NO_KEYS_PRESSED   # go to waiting for no keys to be pressed

                    keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)    # wait debounce time and come back to this function
                    keypadTimer.start()
                '''

        # check if we are waiting for no keys to still be pressed... so that we don't get duplicate keys returned.
        elif (self.__keypadState==I2cKeypad.WAITING_FOR_NO_KEYS_PRESSED):  # this state is used when we are waiting for the last keypress to be released.


            key = self.__checkForKeyPress()
            # change state only if no keys are pressed
            if (key == I2cKeypad.NO_KEYS_PRESSED):
                # we have no keys pressed
                self.__keypadState = I2cKeypad.WAITING_FOR_NEW_KEY_PRESS   # go to waiting for new keys

            keypadTimer = threading.Timer(I2cKeypad.TIMER_PERIOD/1000, self.__scanKeys)    # set up timer to return to this function
            keypadTimer.start()


    # enable hardware interrupts from the mcp23008
    # parameters:
    #    none
    # return:
    #    nothing
    def __mcpEnableInterrupts(self):
        self.__mcpWriteByte(I2cKeypad.MCP_GPINTEN, 0xff)


    # disable hardware interrupts from the mcp23008
    # parameters:
    #    none
    # return:
    #    nothing
    def __mcpDisableInterrupts(self):
        self.__mcpWriteByte(I2cKeypad.MCP_GPINTEN, 0)



    # check keypad for key presses
    # parameters:
    #    none
    # return:
    #    NO_KEYS_PRESSED (-1)  if no keys pressed
    #    MULTIPLE_KEYS_PRESSED (-2)  if multiple keys pressed
    #    else if there is a valid key it returns the value from _keyMap[] for the key that is pressed (0-255)
    #  Note: This code expects that the IODIR register to have the value of _inputPinsMask (which is all 1s for inputs and 0s for the other bits).
    #          and the OLAT register to be set to 0 (all 0s on the output pins).
    #          This so so that we can quickly check for any key presses with the first line of code in this function.
    def __checkForKeyPress(self):
        keyPressed = 0      # set if a key is pressed
        key = I2cKeypad.NO_KEYS_PRESSED            # returned key value
        outputLatch = 0  # value to write to the mcp output latch
        inputPort = 0     # value read from the mcp input pins

        # we do a quick check here to see if any keys are pressed... that way we don't have to do the more complicated check below
        #     in the case where no keys are pressed (which is most of the time).
        # If any GPIO pins (that are configured as inputs) are low, then we must have some switch pressed. If they are all high, then no key is pressed, and we just return.
        #    To test this, we see if this is true (meaning no keys pressed):    !(GPIO port & _inputPinsMask) ^ _inputPinsMask
        #    We read the input port, AND it with  _inputPinsMask and then XOR the result with _inputPinsMask (which checks if any of the inputs pins are not high)
        #    After the ^ XOR operation the result will be non-zero if one of the input pins does not match the mask (meaning some key is pressed). We negate it (using !) to get a 0 result if no key pressed.
        # we do not have a key pressed if the follwing is true (note the negation operator)
        if  not((self.__mcpReadByte(I2cKeypad.MCP_GPIO) & self.__inputPinsMask) ^ self.__inputPinsMask) :
            return I2cKeypad.NO_KEYS_PRESSED


        # for each column, check if any row pins are low
        for col in range(len(self.__colPins)):
            outputLatch = 0xff
            outputLatch = self.__bitClear(outputLatch, self.__colPins[col])      # clear the bit for this column output pin
            self.__mcpWriteByte(I2cKeypad.MCP_IODIR, outputLatch)      # write to the MCP IODIR register to make this one pin an output
                                                       #     We make only one pin be an output so that if multiple keys
                                                       #     are pressed we don't get two output pins shorted together
                                                       #     (and they could be at different levels)
            self.__mcpWriteByte(I2cKeypad.MCP_OLAT, outputLatch)       # write to the MCP output latch to make the one output pin low
            sleep(.001)                                 # wait for the latch signal to propagate to the input pins
            inputPort = self.__mcpReadByte(I2cKeypad.MCP_GPIO)         # read the input port
            # for each row pin, check if the pin is low, meaning a key is pressed for this column/row
            for row in range(len(self.__rowPins)):
                # check if this row pin is low
                if (self.__bitRead(inputPort, self.__rowPins[row]) == 0):
                    # we have a key press at this row and col
                    # check if some other key has already been detected... if so return MULTIPLE_KEYS_PRESSED.
                    if (keyPressed):
                        # Return the MCP to it's quick key checking state, so that when we come back to the function we can quickly check for a key press
                        # Set all input pins to be inputs, and the rest to be outputs...
                        self.__mcpWriteByte(I2cKeypad.MCP_IODIR, self.__inputPinsMask)   # set all input pins to be inputs, and the rest to be outputs
                        # write all output pins to LOW so we can check if any of the input pins are not low later on (meaning a key is pressed)
                        self.__mcpWriteByte(I2cKeypad.MCP_OLAT, 0)
                        return I2cKeypad.MULTIPLE_KEYS_PRESSED

                    # else we have a new key being pressed
                    else:
                        keyPressed = 1
                        key = ord(self.__keyMap[row][col])                # get the key out of the keyMap list of lists, convert from a char to decimal ASCII value



        # Return the MCP to it's quick key checking state, so that when we come back to the function we can quickly check for a key press
        # Set all input pins to be inputs, and the rest to be outputs...
        self.__mcpWriteByte(I2cKeypad.MCP_IODIR, self.__inputPinsMask)   # set all input pins to be inputs, and the rest to be outputs
        # write all output pins to LOW so we can check if any of the input pins are not low later on (meaning a key is pressed)
        self.__mcpWriteByte(I2cKeypad.MCP_OLAT, 0)
        if (keyPressed):
            return key
        else:
            return I2cKeypad.NO_KEYS_PRESSED




    # read one byte from mcp chip register
    # parameters
    #    mcpRegister - the register in the mcp chip that is to be read
    # returns
    #    the the value of the mcp register
    def __mcpReadByte(self, mcpRegister):
        try:
            return i2c.read_byte_data(self.__i2cAddress, mcpRegister)
        except:
            print("error reading from i2c: " + str(self.__i2cAddress))
            pass
        sleep(.001)

    # write a byte to mcp register
    # parameters
    #    mcpRegister - the register in the mcp chip that is to be written to
    #    data - the value to be written to the mcp register
    # returns
    #    nothing
    def __mcpWriteByte(self, mcpRegister, data):
        try:
            i2c.write_byte_data(self.__i2cAddress, mcpRegister, data)
        except:
            print("error writing to i2c: " + str(self.__i2cAddress))
            pass
        sleep(.001)

    # set mcp register bit (0-7) with data (0-1)
    # parameters
    #    mcpRegister - the register in the mcp chip that is to be written to
    #    bit - which bit of the register that is to be written (0-7)
    #    data - the value to be written to the mcp register bit
    # returns
    #    nothing
    def __mcpWriteBit(self, mcpRegister, bit, data):
        mcpRegisterValue = 0    # current value of the mcp register
        if (bit > 7):
            return       # we only have 8 pins, 0-7
        mcpRegisterValue = self.__mcpReadByte(mcpRegister)  # read the current value of the register
        if (data == 1):
            mcpRegisterValue = mcpRegisterValue | (1 << bit)     # set the bit if data==1
        else:
            mcpRegisterValue = mcpRegisterValue & (~(1 << bit))  # clear the bit if data==0
        self.__mcpWriteByte(mcpRegister, mcpRegisterValue)  # rewrite the register


    # read one bit from mcp register (returns 0 or 1)
    # parameters
    #    mcpRegister - the register in the mcp chip that is to be read
    #    bit - the bit of the register to be read
    # returns
    #    the the value of the mcp register bit (0 or 1)
    def __mcpReadBit(self, mcpRegister, bit):
        if (bit > 7):
            return 0       # we only have 8 bits, 0-7
        return (self.__mcpReadByte(mcpRegister) >> bit) & 0x01       # read register, shift to get just desired bit in 0 position, mask it off with 0x01

    # functions for bit manipulation of variables
    def __bitRead(self, value, bit):
        return (((value) >> (bit)) & 0x01)
    def __bitSet(self, value, bit):
        return value | (1 << (bit))
    def __bitClear(self, value, bit):
        return value & ~(1 << (bit))
    def __bitWrite(self, value, bit, bitvalue):
        if (bitvalue == 1):
            return self.__bitSet(value, bit)
        else:
            return self.__bitClear(value, bit)
