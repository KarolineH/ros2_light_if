## Target Hardware
One or multiple ASTORA SF 120 SoftPanels

## Prerequisites
- Install [QLC+](https://www.qlcplus.org/) to control the lights. On Ubuntu run `sudo apt install qlcplus`. Tested on v4.12.2, default with Ubuntu 20.04

## Connecting the lights
1. Connect power cord to the lights
2. Connect the first light to PC using the **DMX-to-USB** cable (you will need a DMX dongle for the small LitraStudio that goes in USB)
3. Chain any extra lights (out>in) with DMX cables.
4. Attach the **terminator** to the output DMX port on the final light panel.
5. Ensure lights work by manually turning knobs located at the back that control intensity and temperature.
6. If you want to control the chained panels inividually, use the channel dial buttons on the back of each light panel to specify separate channels. 
Every panel uses 2 channels to communicate, so set the first light to 1, the next to 3 etc...
