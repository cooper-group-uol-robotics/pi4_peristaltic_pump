from peristaltic_dispenser_driver.DispenserDriver import DispenserDriver
import rospy
from peristaltic_dispenser_driver.msg import DispenserCommand

class DispDriverROS:
    def __init__(self):
        self.dispenser = DispenserDriver()
        rospy.Subscriber("Dispenser_Commands", DispenserCommand, self.callback_commands)
        rospy.loginfo("Dispenser Driver Started")
        
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
        rospy.loginfo("Dispensing Until Stopped at Speed: " + str(speed))
        
    def dispenseIndefinitelyReverse(self, speed):
        self.dispenser.dispenseIndefReverse(speed)
        rospy.loginfo("Dispensing in Reverse Until Stopped at Speed: " + str(speed))
        
    def dispense(self, speed, dispTime):
        self.dispenser.dispense(speed, dispTime)
        rospy.loginfo("Dispensing for " + str(dispTime) + " seconds at Speed: " + str(speed))
        
    def dispenseReverse(self, speed, dispTime):
        self.dispenser.dispendseReverse(speed, dispTime)
        rospy.loginfo("Dispensing in Reverse for " + str(dispTime) + " seconds at Speed: " + str(speed))
    
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
        else:
            rospy.loginfo("Invalid Command")
            
        
