#!/usr/bin/python
"""
Copyright 2019 Shaun Price

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
"""
This webserver drives the RGB LED's with teh priviledges require to communicate with the port.
The ROS application does not have the required priviledges to communicate wit hteh RGB LED's
The default port is 2812, which aligns to teh Neopixel LED controller the ws2812b. 
"""
from flask import Flask
from rpi_ws281x import  Adafruit_NeoPixel, Color
import time
from threading import Thread

# LED strip configuration:
LED_COUNT      = 4      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0

# Initialise Flask
app = Flask(__name__)

class Chasser:
    # Make True to stop
    stopped = False
    # Used to stop the display function to receive the updated command
    update_function = False
    # Time in milliseconds to delay each loop
    wait_ms = 50
    # Function to run
    function = "rgb"
    # Colour to set for functions that take single color value 
    color = Color(0,0,0)
    # Colour for each of the four LEDs
    color1 = Color(0,0,0)
    color2 = Color(0,0,0)
    color3 = Color(0,0,0)
    color4 = Color(0,0,0)

    def __init__(self, strip):
        self.strip = strip
        self.stopped = False

    def start(self):
        Thread(target=self.display, args=()).start()
        return self

    # Signals looping functions to stop
    def stop(self):
        self.stopped = True

    # Delay in milliseconds for each loop
    def delay(self, delay):
        self.wait_ms = int(delay)
        # Tell any running functions to exit
        self.update_function = True

    def update(self,function, color="00000000", led1="000000", led2="000000", led3="000000", led4="000000", delay=50):        
        self.function = function
        self.wait_ms = int(delay)

        if function == "red":
            self.color1 = Color(255,0,0)
            self.color2 = Color(255,0,0)
            self.color3 = Color(255,0,0)
            self.color4 = Color(255,0,0)
        elif function == "green":
            self.color1 = Color(0,255,0)
            self.color2 = Color(0,255,0)
            self.color3 = Color(0,255,0)
            self.color4 = Color(0,255,0)
        elif function == "blue":
            self.color1 = Color(0,0,255)
            self.color2 = Color(0,0,255)
            self.color3 = Color(0,0,255)
            self.color4 = Color(0,0,255)
        else:
            color_red = int(color[0:1],16)
            color_green = int(color[2:3],16)
            color_blue = int(color[4:5],16)

            led1_red = int(led1[0:1],16)
            led1_green = int(led1[2:3],16)
            led1_blue = int(led1[4:5],16)

            led2_red = int(led2[0:1],16)
            led2_green = int(led2[2:3],16)
            led2_blue = int(led2[4:5],16)

            led3_red = int(led3[0:1],16)
            led3_green = int(led3[2:3],16)
            led3_blue = int(led3[4:5],16)

            led4_red = int(led4[0:1],16)
            led4_green = int(led4[2:3],16)
            led4_blue = int(led4[4:5],16)

            self.color = Color(color_red, color_green, color_blue)
            self.color1 = Color(led1_red, led1_green, led1_blue)
            self.color2 = Color(led2_red, led2_green, led2_blue)
            self.color3 = Color(led3_red, led3_green, led3_blue)
            self.color4 = Color(led4_red, led4_green, led4_blue)

        # Tell any running functions to exit
        self.update_function = True

    # Runs as a thread and constantly loops the wait_ms time
    # When the update_function global variable is true the current running function
    # will exit and resset the update_function variable
    # Setting the stop flag to true will exit the display thread
    def display(self):
        while not self.stopped:
            self.update_function = False
            if self.function == "rgb":
                self.rgb()
            elif self.function == "red":
                self.rgb()
            elif self.function == "green":
                self.rgb()
            elif self.function == "blue":
                self.rgb()
            elif self.function == "colorWipe":
                self.colorWipe()
            elif self.function == "theaterChase":
                self.theaterChase()
            elif self.function == "rainbow":
                self.rainbow()
            elif self.function == "theaterChaseRainbow":
                self.theaterChaseRainbow()

            time.sleep(self.wait_ms / 1000.0)

    def rgb(self):
        # Set the colours
        self.strip.setPixelColor(0, self.color1)    # LED 1 colour
        self.strip.setPixelColor(1, self.color2)    # LED 2 colour
        self.strip.setPixelColor(2, self.color3)    # LED 3 colour
        self.strip.setPixelColor(3, self.color4)    # LED 4 colour
        # Display the LEDs
        self.strip.show()
 
    def colorWipe(self):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, self.color)
            self.strip.show()
            if self.stopped or self.update_function: break
            time.sleep(self.wait_ms / 1000.0)

    def wheel(self, pos):
        """Generate rainbow colors across 0-255 positions."""
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)

    def theaterChase(self):
        """Movie theater light style chaser animation."""
        for q in range(3):
            if self.stopped or self.update_function: break
            for i in range(0, strip.numPixels(), 3):
                if self.stopped or self.update_function: break
                self.strip.setPixelColor(i + q, self.color)
            self.strip.show()
            time.sleep(self.wait_ms / 1000.0)
            for i in range(0, strip.numPixels(), 3):
                if self.stopped or self.update_function: break
                self.strip.setPixelColor(i + q, 0)

    def rainbow(self):
        """Draw rainbow that fades across all pixels at once."""
        for j in range(256):
            for i in range(strip.numPixels()):
                if self.stopped or self.update_function: break
                self.strip.setPixelColor(i, self.wheel((i + j) & 255))
            self.strip.show()
            time.sleep(self.wait_ms / 1000.0)

    def theaterChaseRainbow(self):
        """Rainbow movie theater light style chaser animation."""
        for j in range(256):
            if self.stopped or self.update_function: break
            for q in range(3):
                if self.stopped or self.update_function: break
                for i in range(0, strip.numPixels(), 3):
                    if self.stopped or self.update_function: break
                    self.strip.setPixelColor(i + q, self.wheel((i + j) % 255))
                self.strip.show()
                time.sleep(self.wait_ms / 1000.0)
                for i in range(0, strip.numPixels(), 3):
                    if self.stopped or self.update_function: break
                    self.strip.setPixelColor(i + q, 0)

@app.route("/")
def hello():
    return "wsRGB webservice for Waveshare Alphabot2 Robot.\n\n\t \
        Call the setLEG functiuon to set the colours in hex where:\n\n\t \
        rr = red, gg = green, bb = bb.\n\n\t \
        and the format of the string is: \n\n\t\
        /setLED/<LED1>/<LED1>/<LED1>/<LED1>/\n\n\t \
        The following are acceptable:/n/n/t \
        /setLED/rrggbb/rrggbb/rrggbb/rrggbb/\n\n\t \
        Other functions include:\n\n\t \
        /setAllRed/\n\t \
        /setAllGreen/\n\t \
        /setAllBlue/\n\t \
        /setMode/colorWipe/rrggbb/dddd/\n\t where dddd is the delay between loops. \
        /setMode/colorWipe/rrggbb/\n\t Uses the default 50mS delay. \
        /setMode/rainbow/\n\t \
        /setMode/theaterChase/rrggbb/dddd/\n\t where dddd is the delay between loops. \
        /setMode/theaterChaseRainbow/rrggbb/dddd/\n\t where dddd is the delay between loops. \
        /stop/\n\n\t \
        /delay/50/ where 50 is the number of milliseconds to delay\n\n\t"

@app.route("/setLED/<led1>/<led2>/<led3>/<led4>/",methods=['PUT'])
def setLED(led1="000000",led2="000000",led3="000000",led4="000000"):
    chasser.update(function="rgb",led1=led1, led2=led2, led3=led3, led4=led4)
    return "OK\n"

@app.route("/setAllRed/",methods=['PUT'])
def setAllRed():
    # Make sure no chasser functions are running
    chasser.update(function="red")
    return "OK\n"

@app.route("/setAllGreen/",methods=['PUT'])
def setAllGreen():
    chasser.update(function="green")
    return "OK\n"

@app.route("/setAllBlue/",methods=['PUT'])
def setAllBlue():
    chasser.update(function="blue")
    return "OK\n"

@app.route("/setMode/colorWipe/<color>/<delay>/",methods=['PUT'])
def colorWipe(color="000000", delay=50):
    chasser.update(function="colorWipe", color=color, delay=delay)
    return "OK\n"

@app.route("/setMode/theaterChase/<color>/<delay>/",methods=['PUT'])
def theaterChase(color="000000", delay=50):
    chasser.update(function="theaterChase", color=color, delay=delay)
    return "OK\n"

@app.route("/setMode/rainbow/<delay>/",methods=['PUT'])
def rainbow(delay=50):
    chasser.update(function="rainbow",delay=delay)
    return "OK\n"

@app.route("/setMode/theaterChaseRainbow/<delay>/",methods=['PUT'])
def theaterChaseRainbow(delay=50):
    chasser.update(function="theaterChaseRainbow",delay=delay)
    return "OK\n"

@app.route("/setDelay/<delay>/",methods=['PUT'])
def setDelay(delay=50):
    chasser.delay(delay=delay)
    return "OK\n"

if __name__ == "__main__":
    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS,LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    # Create the chaser class
    chasser = Chasser(strip)
    chasser.start()

    app.run(port=2812)

    chasser.stop()