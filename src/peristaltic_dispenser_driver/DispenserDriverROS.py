#!/usr/bin/env python

#ROS Wrapper for a hardware driver for stepper motor based peristaltic pump
#Also launches ROS Kern balance driver, as this is utilized for PID controlled liquid dispensing
#Made by Jakub Glowacki 02/08/2021

import rospy
import roslaunch #launches other required package
import math #Used for weight tolerance
import time
from kern_pcb_balance.msg import KernReading #weight reading from balance
from peristaltic_dispenser_driver.DispenserDriver import DispenserDriver
from peristaltic_dispenser_driver.msg import DispenserCommand
from simple_pid import PID #PID control library
class DispDriverROS:
    def __init__(self):
        global weight
        weight = 0.0 #initialize global weight from balance
        self.dispenser = DispenserDriver() #Create instance of main driver
        rospy.Subscriber("Dispenser_Commands", DispenserCommand, self.callback_commands)
        rospy.Subscriber("/Kern_Weights", KernReading, self.weightCallback)
        rospy.loginfo("Dispenser Driver Started")
        launch = roslaunch.scriptapi.ROSLaunch() #use roslaunch to launch balance driver
        launch.start()
        kernProcess = launch.launch(roslaunch.core.Node('kern_pcb_balance', 'KernPCBROS'))
        rospy.loginfo("Balance Process Launched")
    
    #PID Dispensing function, asks for specified target weight of liquid
    #to be dispensed
    def dispenseLiquid(self, liquidAmount):
        global weight
        startTime = time.time() #Start time defined for timeout
        pid = PID(60, 0.3 ,0, setpoint=liquidAmount) #Begins PID control
        pid.sample_time = 0.2 #Sample time for PID set to balance polling period
        pid.output_limits = (0,1) #Limit output to range of DAC inputs, 0 to 1 as normalized
        targetReached = False
        while (not targetReached): #repeat until target weight reached
            output = pid(weight)
            self.dispenseIndefinitely(output) #use PID process to adjust pump
            #speed in order to dispense correct amount of liquid
            
            #Terminate process successfully if weight is within tolerance
            if (math.isclose(weight, liquidAmount, abs_tol=(0.05))):
                self.reset()
                rospy.loginfo("Liquid dispensed correctly within 0.05g tolerance!")
                targetReached = True
            #Terminate process unsucessfully if weight goes above tolerance,
            #overshoot can't be fixed when dispensing liquid
            if (weight > liquidAmount + 0.05):
                self.reset()
                rospy.loginfo("Overshoot Error! Liquid overshot out of tolerance, check PID tuning")
                targetReached = True
            #Terminate process unsucessfully if target not reached in 60 seconds
            #this could indicate undershoot or another failure
            if (time.time() > startTime + 60):
                self.reset()
                rospy.loginfo("Target not reached in over 60 seconds, check liquid, possible undershoot.")
                targetReached = True
        return True
        
    #Below functions simply switch to the correct functions within
    #the hardware driver as a response to a ROS command
    def on(self):
        self.dispenser.dispenserOn()
        rospy.loginfo("Turning on dispenser")
        
    def off(self):
        self.dispenser.dispenserOff()
        rospy.loginfo("Turning off dispenser")
        
    def reset(self):
        self.dispenser.reset()
        rospy.loginfo("Resetting Dispenser")
    
    def reverse(self):
        self.dispenser.reverseDirection()
        rospy.loginfo("Reversing Dispenser Direction")
    
    def dispenseIndefinitely(self, speed):
        self.dispenser.dispenseIndef(speed)
        rospy.loginfo("Dispensing at Speed: " + str(speed))
        
    def dispenseIndefinitelyReverse(self, speed):
        self.dispenser.dispenseIndefReverse(speed)
        rospy.loginfo("Dispensing in Reverse at Speed: " + str(speed))
        
    def dispense(self, speed, dispTime):
        self.dispenser.dispense(speed, dispTime)
        rospy.loginfo("Dispensing for " + str(dispTime) + " seconds at Speed: " + str(speed))
        
    def dispenseReverse(self, speed, dispTime):
        self.dispenser.dispendseReverse(speed, dispTime)
        rospy.loginfo("Dispensing in Reverse for " + str(dispTime) + " seconds at Speed: " + str(speed))
    
    #callback from receiving weight from balance. Updates corresponding variable.
    def weightCallback(self,msg):
        global weight
        weight = msg.weight
     
    #Callback for receiving commands. Calls the appropriate function for each command.
    def callback_commands(self,msg):
        if(msg.dispenser_command == msg.ON):
            self.on()
        elif(msg.dispenser_command == msg.OFF):
            self.off()
        elif(msg.dispenser_command == msg.RESET):
            self.reset()
        elif(msg.dispenser_command == msg.REVERSE):
            self.reverse()
        elif(msg.dispenser_command == msg.DISPENSEINDEF):
            self.dispenseIndefinitely(msg.dispenser_speed)
        elif(msg.dispenser_command == msg.DISPENSEINDEFREV):
            self.dispenseIndefinitelyReverse(msg.dispenser_speed)
        elif(msg.dispenser_command == msg.DISPENSE):
            self.dispense(msg.dispenser_speed, msg.dispenser_time)
        elif(msg.dispenser_command == msg.DISPENSEREV):
            self.dispenseReverse(msg.dispenser_speed, msg.dispenser_time)
        elif(msg.dispenser_command == msg.DISPENSEPID):
            self.dispenseLiquid(msg.dispenser_ml)                                        
        else:
            rospy.loginfo("Invalid Command")
            
        
