#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import pywapi

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('weather', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        weather = pywapi.get_weather_from_weather_com(u'UKXX1087')
        
        wf = weather['forecasts'][1]
        txt = 'Tomorrow the temperature will be between %s and %s.' % (wf['low'], wf['high'])
        rospy.loginfo(txt)
        pub.publish(txt)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass