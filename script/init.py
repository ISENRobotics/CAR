#!/usr/bin/env python

import os

pwmsitch = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')
pwmservo = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')

print pwmsitch

fichier = open("pwmsitch.txt", "w")
fichier.write("pwmsitch")
fichier.close()

fichier = open("pwmservo.txt", "w")
fichier.write("pwmservo")
fichier.close()