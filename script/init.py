#!/usr/bin/env python

import os

pwmsitch = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')
pwmservo = os.listdir('/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/')

fichier = open("pwmsitch.txt", "w")
fichier.write(pwmsitch[0])
fichier.close()

fichier = open("pwmservo.txt", "w")
fichier.write(pwmservo[0])
fichier.close()
