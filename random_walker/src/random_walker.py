#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg
from strands_navigation_msgs.msg import TopologicalMap
from std_srvs.srv import Empty
from random import shuffle

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
    def __init__(self) :
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
        
    def call_action(self, targ):
        navgoal = topological_navigation.msg.GotoNodeGoal()
    
        print "Requesting Navigation to %s" %targ
    
        navgoal.target = targ
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result(timeout=rospy.Duration(60.0))
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps        

    def cancle_action(self):
        self.client.cancel_all_goals()

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)

class random_walker(object):
    def __init__(self, names) :
        self._running = False
        self._action = topol_nav_client()
        self._names = names
        self._start_server = rospy.Service('/start_random_walk', Empty, self._start_random_walk)
        self._start_server = rospy.Service('/stop_random_walk', Empty, self._stop_random_walk)

    def _start_random_walk(self, req):
        self._running = True
        
    def _stop_random_walk(self, req):
        self._running = False
        self._action.cancle_action()
        
    def run(self):
        while not rospy.is_shutdown():
            for name in names:
                if self._running:           
                    print 'going to ', name
                    self._action.call_action(name)
                    if self._running:
                        rospy.sleep(1.0)
                else:
                    rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('random_walker')
    
    get_names = topol_map_info()
    
    while not get_names.hasNames():
        rospy.sleep(1.0)
    
    names = list(get_names.getNames())
    shuffle(names)
    print names
    
    walker = random_walker(names)
    
    walker.run()
