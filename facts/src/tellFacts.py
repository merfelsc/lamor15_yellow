#!/usr/bin/env python

import rospy
import random

from facts.srv import *

facts = [ 'The first tanks were built in Lincoln [Wikipedia]', 
          'The earliest origins of Lincoln can be traced to the remains of an Iron Age settlement of round wooden dwellings [Wikipedia]',
          'The robot Bob tells jocks but they are not that funny' ]

history = {}

def handle_facts_req(req):
    pid = req.person

    print 'Request for person ', pid

    # add person
    if pid not in history or len(history[pid]) == 0:
        history[pid] = range(len(facts))

    # randomize responses
    fid = random.choice(history[pid])

    del history[pid][history[pid].index(fid)]
    return TellFactsResponse(facts[fid])



def tell_facts_server():
    rospy.init_node('tell_facts_srv')
    s = rospy.Service('tell_facts', TellFacts, handle_facts_req)
    print "Ready to tell facts."
    rospy.spin()


if __name__ == "__main__":
    tell_facts_server()
