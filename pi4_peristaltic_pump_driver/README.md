# pi4_peristaltic_pump_driver
#### ROS Driver for a custom liquid dispenser using a raspberry pi4, peristaltic pump, a DAC and a relay for dispensing,
#### as well as a top pan balance (controlled by kern_pcb_driver ROS driver) used for PID controlled liquid dispensing.
##### Written by Jakub Glowacki and Hatem Fakhruldeen

## Dependencies
Three specific dependencies are required for this to run that cannot be automatically resolved by rosdep. Those are **simple-pid** and **adafruit-circuitpython-mcp4725** both can be installed using either pip (if python 3 is the default environment) or pip3. 
Moreover, the **kern_pcb_balance** ROS package must also be present as it needs to run alongside this driver. This package can be found [here](https://github.com/cooper-group-uol-robotics/kern_pcb_balance)

## How to Launch
The easiest way to launch the package is with roslaunch:
```
roslaunch pi4_peristaltic_pump_driver perstaltic_pump_driver.launch kern_serial_port:=<port_name>
```
This will launch the driver assuming Kern balance is connected to the provided serial port. If no serial port argument is provided, the default port '/dev/ttyUSB0' will be used.

Alternatively, can be launched using rosrun:
```
rosrun pi4_peristaltic_pump_driver perstaltic_pump_driver <kern_serial_port>
```
Similarly, if no serial port argument is provided, the serial port '/dev/ttyUSB0' is used.

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
