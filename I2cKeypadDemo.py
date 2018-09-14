# -*- coding: utf-8 -*-

'''
    I2cKeypadDemo.ino

    Written by: Gary Muhonen  gary@dcity.org

    Versions:
        1.0.0 - 9/1/2018
            Original release.

    Short Description:

        These files provide software for the Raspberry Pi.
        It assumes that you are using an I2C keypad. The keypad
        connects to the I2C bus using a MCP23008 8 bit interface chip.
        A link below details one such backpack board, and there may others.

        A demo program is available that demonstrates most of the features of this library.

        See the project details links below for installation and usage information.

        Github repositories:
        * Arduino library files:  https://github.com/dcityorg/i2c-keypad-library-arduino.git
        * Particle library files: https://github.com/dcityorg/i2c-keypad-library-particle.git
        * Raspberry Pi library files: https://github.com/dcityorg/i2c-keypad-library-raspberrypi.git

        Project Details:

        * Library installation and usage: https://dcity.org/portfolio/i2c-keypad-library/
        * I2C Keypad backpack board: https://dcity.org/portfolio/i2c-keypad-backpack-board/

        I2C Keypad Backpack Board Github Repositories (schematic and board layouts):

        * I2C Keypad Backpack Board https://github.com/dcityorg/i2c-keypad-backpack.git

    https://www.dcity.org/portfolio/i2c-keypad-library/
    This link has details including:
        * software library installation for use with Arduino, Particle and Raspberry Pi boards
        * list of functions available in these libraries
        * a demo program (which shows the usage of most library functions)
        * info on keypads that work with this software
        * hardware design for a backpack board for keypads, available on github
        * info on backpack “bare” pc boards available from OSH Park.

    This demo program is public domain. You may use it for any purpose.
        NO WARRANTY IS IMPLIED.

    License Information:  https://www.dcity.org/license-information/

    Notes:
        1. You must enable I2C on your Raspberry Pi board (see your particular operating system documentation).
            On Raspian: Menu...Preferences...Raspberry Pi Configuration...Interfaces...Enable I2C
        2. This software was tested with Python 3.5.2
        3. The I2cKeypad class uses a software timer from the python "threading" class. If you are not using
            a hardware interrupt from the keypad, then this timer will cause the software to check for a keypress
            every 10 milliseconds.
        4. The user provides a list of lists (__keyMap) that contains ASCII characters (like 'A' or '0').
            When there is a valid keypress, that character and put it into a bytearray (__keyBuffer), which must be
            an unsigned byte value in range of 0-255. The ord() function is used to convert the ASCII chars to the byte value.
            When the user calls a function like getKey(), these byte values are converted back to ASCII characters using the chr() function.
        5. If you are using a hardware interrupt line from the keypad, then this software requires that you have GPIO library version
            0.5.2a or above. See the GPIO import line below.
        6. This software was tested on a RASPBERRY PI 3 MODEL B, running Rasbian and Python 3.

'''



from I2cKeypad import I2cKeypad      # this is the library class for interfacing to a keypad.
import RPi.GPIO as GPIO              # used to cleanup the configuration of GPIO pins upon exiting this program

# Note: you don't have to use interrupts to use this keypad library. If you don't, a timer will be used to frequently check for key presses.
# In this demo code we show both the usage with and without interrupts
KEYPAD_USING_INTERRUPTS = 0         # set this to 1 if you are using the interrupt line from the keypad to trigger an interrupt when a key is pressed
                                    # else, set this to 0
KEYPAD_INTERRUPT_PIN = 23           # set this to the BCM pin number which will be used for interrupts from the keypad...leave it set to 23 even if not using interrupts
                                    # This is the GPIO pin that you are using for interrupts from the keypad.  e.g. BCM23 corresponds to
                                    #    hardware pin 16 on the two rows of pins on the RPi.

KEYPAD_I2C_ADDRESS     = 0x20       # i2c address for the keypad

                            # In this example we are using a 4x4 keypad. Four rows and four columns. You can have up to 8 rows+columns,
                            #    in any mix you want. That is, you could have 3 rows and 4 columns, or 6 rows and 2 columns, but not 6 rows and 3 columns.
rowPins = [0, 1, 2, 3]      # these are the GPx pins on the MCP23008 chip that correspond to the each row pin on the keypad
                            #     the number of items in this list must be exactly the number of rows on your keypad (4 in this example)
                            #     In this example, the first row on the keypad is tied to GP0 on the MCP23008, and so on.
                            # You need to match these pin numbers with the hardware layout of the keypad and how it is wired to the MCP23008 chip.
colPins = [4, 5, 6, 7]      # these are the GPx pins on the MCP23008 chip that correspond to the each column pin on the keypad
                            #     the number of items in this list must exactly match the number of columns on your keypad (4 in this example)
                            #     In this example, the first column on the keypad is tied to GP4 on the MCP23008, and so on.

# This is where you define what (single) character you want returned from the getKey() library function when a key is pressed on the keypad.
# In this example we are using a 4x4 keypad. When the row 1 column 1 key is pressed, then '1' will be returned from the getKey() function.
# The top row of this list corresponds to the top row on the keypad. You can use any ASCII character 1-255 (but not 0, the NUL character)
keyMap = [
  ['1','2','3','A'],                                # row 1 keys, define with the character you want returned
  ['4','5','6','B'],                                # row 2 keys, define with the character you want returned
  ['7','8','9','C'],                                # row 3 keys, define with the character you want returned
  ['0','-','.','E']                                 # row 4 keys, define with the character you want returned
]


'''
# As an example of a keypad that has 2 rows of 5 columns of keys, then you might define these variables something like this:
                            # In this example we are using a 2x5 keypad. Two rows and five columns. You can have up to 8 rows+columns,
                            #    in any mix you want. That is could have 3 rows and 4 columns, or 6 rows and 2 columns, but not 6 rows and 3 columns.
rowPins = [6, 7]            # these are the GPx pins on the MCP23008 chip that correspond to the each row pin on the keypad
                            #     the number of items in this list must be exactly the number of rows on your keypad (2 in this example)
                            #     In this example, the first row on the keypad is tied to GP6 on the MCP23008, and so on.
colPins = [0, 1, 2, 3, 4]   # these are the i/o pins on the MCP23008 chip that correspond to the each column pin on the keypad
                            #     the number of items in this list must be exactly the number of columns on your keypad (5 in this example)
                            #     In this example, the first column on the keypad is tied to GP0 on the MCP23008, and so on.

# This is where you define what (single) character you want returned from the getKey() library function when a key is pressed on the keypad.
# The top row of this list corresponds to the top row on the keypad. You can use any ASCII character 1-255 (zero is reserved).
keyMap = [
  ['0','1','2','3','4'],                                # row 1 keys, define with the character you want returned
  ['A','B','C','X','Y']                                 # row 2 keys, define with the character you want returned
]
'''



# create a keypad object
keypad = I2cKeypad(keyMap, rowPins, colPins, KEYPAD_I2C_ADDRESS, KEYPAD_USING_INTERRUPTS, KEYPAD_INTERRUPT_PIN)    # create an keypad object


# This is the main body of a demo program the uses the I2cKeypad library. It tests most of the functions available in the library.
def main():

    while 1:                         # keep running this program until ctrl C is pressed, this is the main program loop


        # Here we begin a number of small demo segments, that test various functions in the keypad library.
        # Each segment will run on it's own. Or, if you want, you can comment out some segments during your testing.



        # Testing the getKey() function (which returns the next available key in the buffer, if any)
        # This demo reads keys pressed on the keypad and displays them in ASCII, for 10 seconds.
        # Note that the getKey() function does NOT wait for a key to be pressed. It returns ASCII NUL character chr(0) if no key is pressed.
        # This demo repeatedly calls getKey() and uses the keypad.millis() function to determine when 10 seconds is up.
        print('\n')
        print('\n')
        print("Testing getKey()")
        print("For the next 10 seconds, press keys, and they will be displayed...")
        keypad.flushKeys()  # Removes any keys from the keypad buffer. May not be necessary if you don't care if keys were already in the buffer.
        currentTime1 = keypad.millis()
        while (keypad.millis() - currentTime1 < 10000) :
            keyFromKeypad = keypad.getKey()
            if keyFromKeypad != keypad.RETURN_NO_KEY_IN_BUFFER:
                print("This key was pressed: " + keyFromKeypad)



        # Testing the peekKey() function (which returns the next available key in the buffer, without removing it from the buffer).
        # This demo continuously displays the number of keys in the buffer and the first key stored in buffer, for 10 seconds.
        # Note that the peekKey() and getKeyCount() functions do NOT wait for a key to be pressed.
        # This demo repeatedly calls getKeyCount() and uses the keypad.millis() function to see when 10 seconds is up.
        keyCount3 = 0 # this is the current number of keys stored in the private library buffer.
        lastKeyCount3 = 0 # this is the previous checked number of keys stored in the buffer
        currentTime3 = 0  # current keypad.millis() value
        keypad.flushKeys() # Removes any keys from the keypad buffer. May not be necessary if you don't care if keys were already in the buffer.
        print('\n')
        print('\n')
        print("Testing peekKey() and getKeyCount()")
        print("In the next 10 seconds, press some keys on the keypad...")
        currentTime3 = keypad.millis()   # get the current time
        print("Number of keys in buffer is " + str(keyCount3))
        # wait here for 10 seconds
        while (keypad.millis() - currentTime3 < 10000):
            # call the checkKey() function. This function also calls scanKeys() each time, so you don't need to call scanKeys().
            # if the key count is different than last time, then display the new key count and peek at the 1st key stored in the buffer.
            keyCount3 = keypad.getKeyCount()
            if (keyCount3 != lastKeyCount3):
                lastKeyCount3 = keyCount3
                print("Number of keys in buffer is " + str(keyCount3) + "   First key in buffer is " + keypad.peekKey())
                # Since we aren't removing chars from buffer, the first key in buffer is always the same.





        # Testing the getKeyUntil() function (returns when 1 key is pressed, or terminates after timeout period)
        # This function demo returns one ASCII character if a key is pressed, or terminates after 10 seconds.
        # this is the returned ASCII value entered for the key pressed on the keypad. If no key pressed, ASCII character chr(0) is returned.
        keypad.flushKeys() # Removes any keys from the keypad buffer. May not be necessary if you don't care if keys were already in the buffer.
        print('\n')
        print('\n')
        print("Testing getKeyUntil()")
        print("Press one key within 10 seconds...")
        # call the getKeyUntil() function. This function also calls scanKeys() many times, so you don't need to call scanKeys().
        # This function waits for one of the termination conditions to occur, and then returns (i.e. this function blocks other code from running until it is done).
        keyFromKeypad = keypad.getKeyUntil(10000)
        if (keyFromKeypad != keypad.RETURN_NO_KEY_IN_BUFFER):  # if a key is pressed, display it.
            # display the returned ASCII keypad character.
            print("The returned character is: " + keyFromKeypad)
        else: # else binary 0 was returned, so display timeout message.
            print("No key was pressed within 10 seconds.")



if __name__ == "__main__":

    try:
        main()

    finally:                   # run on exit
        GPIO.cleanup()         # clean up GPIO configurations
        print('\n')
        print ("Exiting, GPIO cleaned up.")
        print('\n')
