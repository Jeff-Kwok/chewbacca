#!/usr/bin/env python3
import time
import busio
import board
from adafruit_pca9685 import PCA9685

# --- PCA SETUP ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# --- PICK ONE WHEEL HERE ---
# Example: nfr = [dirA=4, dirB=5, pwm=9]
DIR_A = 0
DIR_B = 8 
PWM   = 8

# --- TUNE THESE ---
PWM_DUTY = 20000   # try 12000, 20000, 40000, 65535
ON_SEC   = 2.0
OFF_SEC  = 1.0

def stop():
    pca.channels[DIR_A].duty_cycle = 0
    pca.channels[DIR_B].duty_cycle = 0
    pca.channels[PWM].duty_cycle = 0

def forward(duty):
    pca.channels[DIR_A].duty_cycle = 0xFFFF
    pca.channels[DIR_B].duty_cycle = 0x0000
    pca.channels[PWM].duty_cycle   = duty

def reverse(duty):
    pca.channels[DIR_A].duty_cycle = 0x0000
    pca.channels[DIR_B].duty_cycle = 0xFFFF
    pca.channels[PWM].duty_cycle   = duty

try:
    print("STOP")
    stop()
    time.sleep(OFF_SEC)

    print(f"FORWARD duty={PWM_DUTY}")
    forward(PWM_DUTY)
    time.sleep(ON_SEC)

    print("STOP")
    stop()
    time.sleep(OFF_SEC)

    print(f"REVERSE duty={PWM_DUTY}")
    reverse(PWM_DUTY)
    time.sleep(ON_SEC)

    print("STOP")
    stop()
    time.sleep(OFF_SEC)

    print("DONE")
finally:
    stop()
    pca.deinit()
