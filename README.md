This is a ROS2 (Humble) Wrapper for the Python Interface located [here](https://github.com/KarolineH/dmx_light_interface/tree/d5cdf891cd73561ab6cc05bbc31e9051774320af).\
It can be used to control intensity and colour temperature of one or multiple ASTORA light fixtures via DMX. \

## Target Hardware
One or multiple ASTORA SF 120 SoftPanels

## Prerequisites & Installation
- Install [QLC+](https://www.qlcplus.org/) to control the lights. On Ubuntu run `sudo apt install qlcplus`. Tested on v4.12.2, default with Ubuntu 20.04
- To use this ROS package, make sure to clone this repo WITH its submodule `git clone --recurse-submodules https://github.com/KarolineH/ros2_light_if`

## Connecting the lights
1. Connect power cord to the lights
2. Connect the first light to PC using a **DMX-to-USB** cable (you might need a DMX dongle)
3. Chain any additional lights (out>in) with DMX cables.
4. Attach a **DMX terminator** to the output DMX port on the final light panel.
5. Ensure lights work by manually turning knobs located at the back that control intensity and temperature.
6. If you want to control the chained panels inividually, use the channel dial buttons on the back of each light panel to specify separate channels. 
Every panel uses 2 channels to communicate, so set the first light to 1, the next to 3 etc...
