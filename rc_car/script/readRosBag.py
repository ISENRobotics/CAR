#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference

def callback(data):
    rospy.loginfo("lat [%f], lon [%f], status [%i]", data.latitude, data.longitude, data.status.status)
    with open('dataBlueTeam.txt','a') as f: f.write('{0} {1} {2}\n'.format(data.latitude, data.longitude, data.status.status))
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenerRosbag')

    rospy.Subscriber("fix", NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
