# Low Level Control

There are two components in the low level control i.e. the motor controller and the central hub. They are both implemented on the Raspberry Pi Pico using micropython.


## Loading/Running code on Pi Pico (IMPORTANT)
To load the code onto the Pi Pico, it is recommended to use Thonny IDE as they provide a GUI for the filesystem on the Pico. You can also use the command line tools (e.g. mpremote, rshell) to load the code onto the Pico.

First, load the common `user_interface` folder and `protocol.py` file onto the pico then load either the `motor_controller` or `central_hub` folder onto the pico depends on which component you want to run. Then, add respective `main.py` file to the root directory of the pico.

## Motor Controller
The motor controller is responsible for controlling the motors and reading off the sensors attached to the motor i.e. encodes and current sensors. It controls the motor using Sabertooth 2x12A through packetised serial protocol. The current sensors (ACS711EX) feeds ADC into the Pi Pico. The encoders are decoded in hardware using the Pi Pico's PIO.

## Central Hub
The central hub is responsible for controlling the modes of operation and the communication between the motor controller and the flight controller. This implementation offers three levels of autonomy i.e. direct RC control, Flight controller aided and fully autonomous mode. In all of these modes the Radio controller must be connected at all time.
