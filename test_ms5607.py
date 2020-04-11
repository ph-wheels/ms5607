
import machine
from machine import I2C
import time

from ms5607 import MS5607

i2c = machine.I2C(sda=machine.Pin(23),scl=machine.Pin(22)) # pin usage may vary on your hardware

address = 0x77 # or 0x76 based on alternative address pin
samples = 64
ms = MS5607(i2c,address) # has optional 3th parameter True for debug mode
alt = ms.get_altitude(samples)
print ('Heigth above MSL ->  %6.2f Mtr' % float(alt/1000))
hpa = ms.get_pressure(samples)
print ('Air pressure ->     %6.2f HPa' % float(hpa/100))
tmp = ms.get_temperature(samples)
print ('Temperature ->       %6.2f Cel' % float(tmp/100))