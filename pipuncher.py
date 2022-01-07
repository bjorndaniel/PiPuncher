#!/usr/bin/python3

import smbus2 as smbus
import math
import time
import RPi.GPIO as GPIO

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Set up button and led
GPIO.setmode(GPIO.BOARD)
ledPin = 12
buttonPin = 18
GPIO.setup(ledPin, GPIO.OUT)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(ledPin, GPIO.LOW)

# Set up accelerator
bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command
# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def read_accel(pin):
    try:
        GPIO.remove_event_detect(buttonPin)
        print("PUNCH!")
        GPIO.output(ledPin, GPIO.HIGH)
        t_end = time.time() + 5
        baseValue = read_word_2c(0x3b)
        baseValue =  baseValue / 16384.0
        readings = []
        delta = 0.0
        while time.time() < t_end:
            tempVal = read_word_2c(0x3b)
            readings.append(tempVal/16384.0)
            time.sleep(0.05)
        for i in readings:
            tempDel = abs(baseValue - i)
            if(tempDel > delta):
                delta = tempDel
        GPIO.output(ledPin, GPIO.LOW)
        print("Delta value: ", delta * 100)
        GPIO.add_event_detect(buttonPin, GPIO.FALLING, callback=read_accel)
    except:
        print("Something went wrong, try again!")
        GPIO.output(ledPin, GPIO.LOW)
        GPIO.add_event_detect(buttonPin, GPIO.FALLING, callback=read_accel)
    return

#Main   
GPIO.add_event_detect(buttonPin, GPIO.FALLING, callback=read_accel)
while 1:
    time.sleep(0.5)