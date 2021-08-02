# pi4_peristaltic_pump
#### ROS Driver for a custom liquid dispenser using a peristaltic pump, a DAC and a relay for dispensing,
#### as well as a top pan balance (controlled by kern_pcb_driver ROS driver) used for PID controlled liquid dispensing.
##### Written by Jakub Glowacki

## Dependencies
Two specific dependencies are required for this to run that cannot be automatically resolved by rosdep. Those are **simple-pid** and **adafruit-circuitpython-mcp4725** both can be installed using either pip (if python 3 is the default environment) or pip3. 
The kern_pcb_driver ROS package must also be present as it needs to run alongside this driver.

## How to Launch
The easiest way to launch the package is with roslaunch:
```
roslaunch peristaltic_dispenser_driver DriverROS.launch
```
Alternatively, can be launched using rosrun:
```
rosrun peristaltic_dispenser_driver DispenserDriverROS
```

## ROS Topics:
Dispenser_Commands | For publishing commands to\

## How to send Commands:
Commands are sent using the **Dispenser_Commands** topic. Each command has its own command ID which can be sent through the topic as a **dispenser_command**. A dispenser_command is just a simple integer corresponding to a command, as indicated by the list below. Additionally, some of the commands support additional fields. These are **dispenser_speed**, **dispenser_time** and **dispenser_ml**. If these are passed to commands that don't require it they will be ignored, and if not passed where a command expects them to be, they will be automatically assumed to be zero. The example command below will use PID control to dispense 54.69ml:
```
rostopic pub -1 /Dispenser_Commands peristaltic_dispenser_driver/DispenserCommand '{dispenser_command: 8, dispenser_ml: 54.69}'

```

## Command List:
0 | Manually turns pump on\
1 | Manually turns pump off\
2 | Turns pump off, resets all other values back to their default value; resets system\
3 | Manually reverses the flow of the liquid/direction of pump\
4 | Dispenses in the forward direction indefinitely, using speed specified using **dispenser_speed** as a **float** ranging from 0 (stopped) to 1 (maximum speed)\
5 | Dispenses in the reverse direction indefinitely, using speed specified using **dispenser_speed** as a **float** ranging from 0 (stopped) to 1 (maximum speed)\
6 | Dispenses in the forward direction for amount of time specified using **dispenser_time** as an **integer** corresponding to the number of seconds for which to dispense, using speed specified using **dispenser_speed** as a **float** ranging from 0 (stopped) to 1 (maximum speed)\
7 | Dispenses in the reverse direction for amount of time specified using **dispenser_time** as an **integer** corresponding to the number of seconds for which to dispense, using speed specified using **dispenser_speed** as a **float** ranging from 0 (stopped) to 1 (maximum speed)\
**8 | Dispenses using PID control, amount of liquid is specified using dispenser_ml as a float corresponding to the weight in mililiters of the liquid to be dispensed.**
