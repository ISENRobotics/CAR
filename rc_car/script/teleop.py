#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from rc_car.msg import Command
from rc_car.msg import SwitchMsg

import sys, select, termios, tty

msg = """
Control Your CAR!
---------------------------
Moving around:   m : switch mode
        z     
   q    s    d
  

f/g : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

avance = {
        'z':1,
        's':-1
         }

tourne = {
        'q':45,
        'd':-45
         }

speedBindings={
        'f':(1.1,1.1),
        'g':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop')
    pub = rospy.Publisher('tCommand', Command, queue_size=1)
    pubmode = rospy.Publisher('tSwitchMode', SwitchMsg, queue_size=1)

    mode = SwitchMsg()
    mode.modeAuto=False
    x = 0
    y = 0
    status = 0
    count = 0
    count1 = 0
    count2 = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in avance.keys():
                x = avance[key]
                #print msg
                #print x
                count1 = 0
            elif key in tourne.keys():
                y = tourne[key]
                #print msg
                #print x
                count2 = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                control_speed = 0
                control_turn = 0
            elif key == 'm' :
                if mode.modeAuto :
                    mode.modeAuto=False
                else :
                    mode.modeAuto=True

                pubmode.publish(mode)
                print "modeauto : %d " % (mode.modeAuto)
            else:
                count = count + 1
                count1 = count1 + 1
                count2 = count2 + 1
                if count1 > 5:
                    x = 0
                if count2 > 10:
                    y = 0
                if (key == '\x03'):
                    break
            target_speed = speed * x
            target_turn = turn * y

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed
            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 4.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 4.5 )
            else:
                control_turn = target_turn
            #twist = Twist()
            #twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            #pub.publish(twist)
            #print vels(x,y)
            #print "-------" 
            #print vels(control_speed,control_turn)


            command = Command()
            command.speed = control_speed
            command.dir = control_turn
            pub.publish(command)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e

    finally:
         command = Command()
         command.speed = 0             
         command.dir = 0           
         pub.publish(command)
        #twist = Twist()
        #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

