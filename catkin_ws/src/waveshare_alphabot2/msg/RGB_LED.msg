# RGB LED functions and values for the Waveshare robot
string function	    # Function can be one of setLED, setAllRed, setAllGreen, setAllBlue, colorWipe, rainbow, theaterChase, theaterChaseRainbow, delay
string color        # When using colorWipe or theaterChase functions this is the color of the LED's 
string led1_color  # When using setLED this is the color of the first LED (1 is the robots left LED)
string led2_color  # When using setLED this is the color of the second LED (2 is the robots right-middle LED)
string led3_color  # When using setLED this is the color of the third LED (3 is the left-middle LED)
string led4_color  # When using setLED this is the color of the forth LED (4 is the robots left LED)
uint16 delay        # Delay between loops (i.e. the speed of the display)