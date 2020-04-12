MS5607.py
=====================
A micro python port of the MS5607 wrapper found here:
which has been tested on ESP32

Main source for device specific infomation can be found on
https://www.te.com -> AN520

https://github.com/rsolomon/py-MS5607
which is based on
https://github.com/cypherkey/RaspberryPi.Net

version
---------------------
0.0.2

revision
--------------------
Added a coeficient prom crc test function

Added a function to obtain temperature & pressure

Added a function for quick test

usage
--------------------
execute test_ms5607.py

Required imports:
--------------------
Import machine

From machine import I2C

import time
