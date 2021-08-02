#!/usr/bin/env python

#Hardware driver for stepper motor based peristaltic pump
#Utilizes an MCP4725 DAC for speed control, as well as a relay for switching
#Made by Jakub Glowacki 02/08/2021

from gpiozero import LED #controls GPIO
import board
import busio
import adafruit_mcp4725 #controls adafuit DAC
import time

class DispenserDriver:
        global direct #Direction of pump boolean, default is false, used to reverse pump
        direct = False
        def __init__(self):
            global dac
            global stepper
            global direction
            i2c = busio.I2C(board.SCL, board.SDA) #Create I2C connection with DAC
            dac = adafruit_mcp4725.MCP4725(i2c, address=0x60)
            dac.value = 0 #DAC value set to 0 so pump is not running
            stepper = LED(17) #Connect to both GPIO pins needed for pump control
            direction = LED(27)
            stepper.on() #on=off due to relay wiring, turn pump off by default
            direction.off()
            direct=False
            
        def dispenserOn(self): #Simply turn pump on
            global stepper
            stepper.off()
        
        def reset(self): #Set speed back to 0, direction to default off and turn pump off
            global stepper
            global direction
            global dac
            stepper.on()
            direction.off()
            dac.value=0
            
        def reverseDirection(self): #Reverse the direction of the pump
            global direction
            global direct
            if (direct):
                direction.off()
                direct=False
            else:
                direction.on()
                direct=True
        
        def dispenserOff(self): #Turn off pump
            global stepper
            stepper.on()
            
        def dispenseIndef(self, speed): #Turn pump on at normalized speed (0 to 1).
                                        #Will keep running until turned off
            global stepper
            global dac
            global direction
            stepper.off()
            direction.off()
            direct=False
            dac.normalized_value=speed
        
        def dispenseIndefReverse(self, speed): #Same as above but in other direction
            global stepper
            global dac
            global direction
            stepper.off()
            direction.on()
            direct=True
            dac.normalized_value=speed
        
        def dispense(self, speed, dispTime): #Turn pump on for specified amount of time
            global stepper
            global dac
            global direction
            stepper.off()
            direction.off()
            direct=False
            dac.normalized_value=speed
            time.sleep(dispTime)
            stepper.on()
            dac.normalized_value=0
            
        def dispenseReverse(self, speed, dispTime): #Same as above but in reverse
            global stepper
            global dac
            global direction
            stepper.off()
            direction.on()
            direct=True
            dac.normalized_value=speed
            time.sleep(dispTime)
            stepper.on()
            dac.normalized_value=0
        
        
            
        