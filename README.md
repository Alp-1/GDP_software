
![banner-nana](https://github.com/user-attachments/assets/7941604b-cee3-448b-9373-da8d395b1adb)

This project aims to develop a rover that can navigate through different terrain in a forest environment autonomously.
The wiki containing more informatin about the rover can be found [here](https://github.com/Alp-1/NANA-Forest-Rover/wiki).

A more detailed account of the development process can be found [here](https://alportakci.com/projects/).

## The directory layout

    .
    ├── .vscode                 # VS Code settings
    ├── CAD Models              # CAD Models used in the project with attributions. 
    ├── ESP32                   # ESP32 code for ESP32-C3 dronebridge.
    ├── GPS                     # Script to start NTRIP caster.
    ├── low_level_control       # Low level control
    ├── lua_scripts             # Lua scripts uploaded in flight controller
    ├── navigation              # Autonomous navigation
    ├── pyrealsense             # Intel Realsense library required on Raspberry Pi
    ├── schematic               # Schematic of the low level controller
    ├── .gitignore
    ├── pyproject.toml          # Python dependencies
    └── README.md

### Installation
To run some code that are not hardware dependent on your local PC, you can run the following commands in Python (Version used: **3.10.5** in PC and Raspberry Pi):

(./ is the repo root directory)
```sh
$ pip install -e .
```

## Low Level Control
There are two components in the low level control i.e. the motor controller and the central hub. They are both implemented on the Raspberry Pi Pico using micropython. See the [README.md](low_level_control/README.md) in the low level control folder for more information.

## Autonomous navigation
The files in the pyrealsense folder may have to be in the same directory as the navigation scripts, depending on your installation.
Another option is to include the right package from local directory i.e. 
```python
from pyrealsense import pyrealsense2 as rs
``` 
To run the autonomous navigation script, use the following command in the navigation folder:
```sh
$ python navigation_final.py
```
## Research Poster

![Poster](https://github.com/user-attachments/assets/2642907a-6847-46e5-974c-356a01847da1)

