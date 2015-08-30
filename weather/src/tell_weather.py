#!/usr/bin/env python

import rospy
import pywapi

from weather.srv import *


def handle_weather_req(req):
    weather = pywapi.get_weather_from_weather_com(u'UKXX1087')
    wf      = weather['forecasts'][1]
    txt     = 'Tomorrow the temperature will be between %s and %s.' % (wf['low'], wf['high'])
    return TellWeatherResponse(txt)



def tell_weather_server():
    rospy.init_node('tell_weather_srv')
    s = rospy.Service('tell_weather', TellWeather, handle_weather_req)
    print "Ready to tell weather."
    rospy.spin()


if __name__ == "__main__":
    tell_weather_server()
