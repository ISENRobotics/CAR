#!/usr/bin/env python

import os

pwmsitch = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')
pwmservo = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')

print pwmsitch