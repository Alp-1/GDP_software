# Lua Script

This folder contains the lua script used in this project

## Scripts
[rover-TerrainDetector.lua](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/rover-TerrainDetector.lua) is used to slow down the rover when it encounters rough terrain.

- change the 'SPEED' parameter to change the value of speed of rough terrain. 

[rover-quicktune.lua](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/rover-quicktune.lua) is used to tune the steering and speed controller gains for rover.
- The rover needs to be put in CIRCLE flight mode which is not achievable with QGround Control. Mission Planner is needed in this scenario.




