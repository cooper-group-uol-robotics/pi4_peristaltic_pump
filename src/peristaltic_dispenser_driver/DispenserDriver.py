from gpiozero import LED
import board
import busio
import adafruit_mcp4725
import time

class DispenserDriver:
        global direct
        direct = False
        def __init__(self):
            global dac
            global stepper
            global direction
            i2c = busio.I2C(board.SCL, board.SDA)
            dac = adafruit_mcp4725.MCP4725(i2c, address=0x60)
            dac.value = 0
            stepper = LED(17)
            direction = LED(27)
            stepper.on()
            direction.off()
            direct=False
            
        def dispenserOn(self):
            global stepper
            stepper.off()
        
        def reset(self):
            global stepper
            global direction
            global dac
            stepper.on()
            direction.off()
            dac.value=0
            
        def reverseDirection(self):
            global direction
            global direct
            if (direct):
                direction.off()
                direct=False
            else:
                direction.on()
                direct=True
        
        def dispenserOff(self):
            global stepper
            stepper.on()
            
        def dispenseIndef(self, speed):
            global stepper
            global dac
            global direction
            stepper.off()
            direction.off()
            direct=False
            dac.normalized_value=speed
        
        def dispenseIndefReverse(self, speed):
            global stepper
            global dac
            global direction
            stepper.off()
            direction.on()
            direct=True
            dac.normalized_value=speed
        
        def dispense(self, speed, dispTime):
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
            
        def dispenseReverse(self, speed, dispTime):
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
        
        
            
        