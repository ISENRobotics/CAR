#!/usr/bin/env python


import rospy

from rc_car.msg import SwitchMsg
from rc_car.msg import InterfaceMsg

import sys, select, termios, tty

msg = """
RC CAR!
---------------------------
  m : switch mode



CTRL-C to quit
"""

def callbackint(data):
    rospy.loginfo("nb waypoint : %s   waypoint en cours : %s", data.waypointTotal,data.waypointEnCours)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key





if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('interface')
    pubmode = rospy.Publisher('tSwitchMode', SwitchMsg, queue_size=1)
    rospy.Subscriber("tInterface", InterfaceMsg, callbackint)

    mode = SwitchMsg()
    mode.modeAuto=False
    
    try:
        print msg
        while(1):
            key = getKey()
            if key == 'm' :
                if mode.modeAuto :
                    mode.modeAuto=False
                else :
                    mode.modeAuto=True

                pubmode.publish(mode)
                print "modeauto : %d " % (mode.modeAuto)
            else:
                if (key == '\x03'):
                    break
   

    except:
        print e

    finally:
        mode.modeAuto=False

        pubmode.publish(mode)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

