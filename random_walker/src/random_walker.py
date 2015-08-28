#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg
from strands_navigation_msgs.msg import TopologicalMap

class topol_map_info(object):
    def __init__(self) :
        self.sub = rospy.Subscriber("/topological_map", TopologicalMap, self._callback)
        rospy.loginfo(" ... done with subscribing map")
        self._node_names = None

    def _callback(self, msg):
        if self._node_names is None :
            self._node_names = set([n.name for n in msg.nodes])
            self._node_names.remove('ChargingPoint')
            
                
    def hasNames(self):
        return not self._node_names is None
        
    def getNames(self):
        return self._node_names

class topol_nav_client(object):
    
    def __init__(self, targ) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        navgoal = topological_navigation.msg.GotoNodeGoal()
    
        print "Requesting Navigation to %s" %targ
    
        navgoal.target = targ
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('random_walker')
    
    get_names = topol_map_info()
    
    while True:
        if not get_names.hasNames():
            rospy.sleep(1.0)
        else:
            break
    
    names = get_names.getNames()
    
    print names
    
    while not rospy.is_shutdown():
        for name in names:
            print 'going to ', name
            topol_nav_client(name)
            rospy.sleep(10.0)
