# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#! /usr/bin/python3
import time
import board
import adafruit_mpu6050
import math
import serial
bluePill = serial.Serial(port=r'/dev/ttyS0',  baudrate=9600, timeout=.15)

AVERAGE_OVER_MS = 800
SAMPLE_EVERY_MS = 10
UP_TO_UP_DELAY_MS = 15000 #must wait 15 seconds after sending an up signal to send another one
DOWN_TO_DOWN_DELAY_MS = 2000 # must wait 2 seconds after sending a down signal to send another one

lastUp = 0
lastDown = 0

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
mpu = adafruit_mpu6050.MPU6050(i2c)
gyroY = []
movingDegS = 0
degrees = 0

while (len(gyroY) < AVERAGE_OVER_MS/SAMPLE_EVERY_MS):
    try:
        gyroY.append(mpu.gyro[1])
        time.sleep(SAMPLE_EVERY_MS/1000)
    except OSError as error:
        print(error)
        continue


while True:
    try:
        gyroY.append(mpu.gyro[1])
        gyroY.pop(0)
    except OSError as error:
        print(error)
        continue
    
    movingDegS = sum(gyroY)/len(gyroY)*180/math.pi -2.2
    
    degrees = movingDegS*AVERAGE_OVER_MS/1000
    print(degrees)
    if(degrees < -5):
        if(round(time.time() * 1000) -  lastUp > UP_TO_UP_DELAY_MS):
            lastUp = round(time.time()*1000)
            bluePill.write(bytes("2",  'utf-8'))
            print("UP Ramp Detected")
    elif(degrees > 5 ):
        if(round(time.time()*1000) - lastDown > DOWN_TO_DOWN_DELAY_MS):
            lastDown = round(time.time()*1000)
            bluePill.write(bytes("1", 'utf-8'))
            print("DOWN RAMP DETECTED")
    
    time.sleep(SAMPLE_EVERY_MS/1000)


