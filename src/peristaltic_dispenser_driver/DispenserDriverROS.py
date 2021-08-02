import rospy
import roslaunch
import math
import time
from kern_pcb_balance.msg import KernReading
from peristaltic_dispenser_driver.DispenserDriver import DispenserDriver
from peristaltic_dispenser_driver.msg import DispenserCommand
from simple_pid import PID
class DispDriverROS:
    def __init__(self):
        global weight
        weight = 0.0
        self.dispenser = DispenserDriver()
        rospy.Subscriber("Dispenser_Commands", DispenserCommand, self.callback_commands)
        rospy.Subscriber("/Kern_Weights", KernReading, self.weightCallback)
        rospy.loginfo("Dispenser Driver Started")
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        kernProcess = launch.launch(roslaunch.core.Node('kern_pcb_balance', 'KernPCBROS'))
        rospy.loginfo("Balance Process Launched")
        
    def dispenseLiquid(self, liquidAmount):
        global weight
        startTime = time.time()
        pid = PID(0.1,0.5,0.01, setpoint=liquidAmount)
        pid.sample_time = 0.2
        pid.output_limits = (0,1)
        targetReached = False
        while (not targetReached):
            output = pid(weight)
            self.dispenseIndefinitely(output)
            if (math.isclose(weight, liquidAmount, abs_tol=(0.05))):
                self.reset()
                rospy.loginfo("Liquid dispensed correctly within 0.05g tolerance!")
                targetReached = True
            if (weight > liquidAmount + 0.05):
                self.reset()
                rospy.loginfo("Overshoot Error! Liquid overshot out of tolerance, check PID tuning")
                targetReached = True
            if (time.time() > startTime + 30):
                self.reset()
                rospy.loginfo("Target not reached in over 30 seconds, check liquid, possible undershoot.")
                targetReached = True
        return True
        
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
    
    def weightCallback(self,msg):
        global weight
        weight = msg.weight
        
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
            
        
