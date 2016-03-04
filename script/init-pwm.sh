#!/bin/bash -e
#
# Copyright (c) 2015 Robert Nelson <robertcnelson@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

if ! id | grep -q root; then
	echo "must be run as root"
	exit
fi

ecapvar=`cat ./pwmswitch.txt`
ehrpwmvar=`cat ./pwmservo.txt`

switch="/sys/class/pwm/${ecapvar}/pwm0"
servo="/sys/class/pwm/${ehrpwmvar}/pwm0"
dc="/sys/class/pwm/${ehrpwmvar}/pwm1"

arch=$(uname -m)

if [ "x${arch}" != "xarmv7l" ] ; then
	echo ""
	echo "Error: this script: [$0] is not supported to run under [${arch}]"
	echo "-----------------------------"
	exit
fi

pwm_export () {
	if [ ! -d ${switch} ] ; then
		echo 0 > /sys/class/pwm/${ecapvar}/export || true
	fi
	if [ ! -d ${servo} ] ; then
		echo 0 > /sys/class/pwm/${ehrpwmvar}/export || true
	fi
	if [ ! -d ${dc} ] ; then
		echo 1 > /sys/class/pwm/${ehrpwmvar}/export || true
	fi
chmod 646 /sys/class/pwm/${ecapvar}/pwm0/*
chmod 646 /sys/class/pwm/${ehrpwmvar}/pwm0/*
chmod 646 /sys/class/pwm/${ehrpwmvar}/pwm1/*
}



pwm_enable () {
#period - The total period of the PWM signal (read/write).
#	Value is in nanoseconds and is the sum of the active and inactive
#	time of the PWM.

#duty_cycle - The active time of the PWM signal (read/write).
#	Value is in nanoseconds and must be less than the period.

	pwm="$1"
	if [ -d ${pwm} ] ; then
		if [ "x${period}" = "x" ] && [ "x${duty_cycle}" = "x" ] ; then
			echo 1 > "${pwm}/enable"
		else
			echo 0 > "${pwm}/enable"
			echo ${period} > "${pwm}/period"
			echo ${duty_cycle} > "${pwm}/duty_cycle"
			echo 1 > "${pwm}/enable"
			unset period
			unset duty_cycle
		fi
	fi
}

pwm_disable_simple () {
	pwm="$1"
	if [ -d ${pwm} ] ; then
		echo 0 > "${pwm}/enable"
	fi
}

pwm_disable () {
	pwm="$1"
	if [ -d ${pwm} ] ; then
		echo 0 > "${pwm}/enable"
		#kill duty_cycle, as pwmchip1 pwm0/pwm1 seem to leak on each other...
		echo 0 > "${pwm}/duty_cycle"
	fi
}

pwm_disable_all () {
	pwm_disable_simple ${dc}
	sleep 1
	pwm_disable_simple ${servo}
	sleep 1
	pwm_disable_simple ${switch}
	sleep 1
}

echo "PWM: exporting pwm..."
pwm_export
sleep 1

echo "PWM: all PWM's off"
pwm_disable_all

#p9_42
#chip0 pwm0
echo "PWM: red led"
period="14000000"
duty_cycle="0" 
pwm_enable ${switch}
sleep 1
#pwm_disable ${red}
sleep 1


#p9_14
#chip1 pwm0
echo "PWM: green led"
period="14000000"
duty_cycle="1300000" #1,34ms 0servo
pwm_enable ${servo}
sleep 1
#pwm_disable ${green}
sleep 1


#p9_16
#chip1 pwm1
echo "PWM: blue led"
period="14000000"
duty_cycle="1480000" #1,46ms 0motorDC
pwm_enable ${dc}
sleep 1
#pwm_disable ${blue}
sleep 1

#echo "PWM: all PWM's off"

