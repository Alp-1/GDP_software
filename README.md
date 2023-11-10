# G16 GDP - Autonomous Forest Navigation Rover

This project aims to develop a rover that can navigate through different terrain in a forest environment autonomously.

Each folder in this repo represents a component in our system.

## Low Level Control

There are two components in the low level control i.e. the motor controller and the central hub. They are both implemented on the Raspberry Pi Pico using micropython.

### Installation
To run some code that are not hardware dependent on your local PC, you can run the following commands in Python (Version used: 3.11.4):

(The ./ is the root directory of the repo)
```sh
$ pip install -e .
```

### Running tests
The tests are implemented using pytest. To run the tests, run the following command in the root directory of the repo:
```sh
$ pytest
```

### Loading/Running code on Pi Pico
To load the code onto the Pi Pico, it is recommended to use Thonny IDE as they provide a GUI for the filesystem on the Pico. You can also use the command line tools (e.g. mpremote, rshell) to load the code onto the Pico.

First, load the common `user_interface` folder onto the pico then load either the `motor_controller` or `central_hub` folder onto the pico depends on which component you want to run. Then, add respective `main.py` file to the root directory of the pico.


License
----

MIT
