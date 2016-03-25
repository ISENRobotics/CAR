#!/usr/bin/python

import os

pwmswitch = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')
pwmservo = os.listdir('/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/')

fichier = open("pwmswitch.txt", "w")
fichier.write(pwmswitch[0])
fichier.close()

fichier = open("pwmservo.txt", "w")
fichier.write(pwmservo[0])
fichier.close()
