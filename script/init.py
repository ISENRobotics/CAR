#!/usr/bin/env python

import os

pwmsitch = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')
pwmservo = os.listdir('/sys/devices/platform/ocp/48300000.epwmss/48300100.ecap/pwm/')

print pwmsitch

fichier = open("pwmsitch.txt", "w")
fichier.write(pwmsitch)
fichier.close()

fichier = open("pwmservo.txt", "w")
<<<<<<< HEAD
fichier.write("pwmservo")
fichier.close()
=======
fichier.write(pwmservo)
fichier.close()
>>>>>>> a76bde086974eecd0e62587ca4b8d92ac8a19722
