#!/usr/bin/env python

import rospy
import random
import socket

from urllib2 import Request, urlopen
from facts.srv import *

socket.setdefaulttimeout(1)

facts = [ 'Fact 1', 
          'Fact 2' ]

history = {}

def handle_facts_req(req):
    pid = req.person

    print 'Request for person ', pid

    # just try to fetch a chuck noris joke if offline this just throws an exception and normal
    # handling will take over
    if pid in history and len(history[pid]) == 0:
        # never eval arbitrary website content ;)
        try:
            joke = eval(urlopen('http://api.icndb.com/jokes/random', None, 1).read())['value']['joke']
            return TellFactsResponse(joke)
        except:
            pass

    # add person
    if pid not in history or len(history[pid]) == 0:
        history[pid] = range(len(facts))

    # randomize responses
    fid = random.choice(history[pid])

    del history[pid][history[pid].index(fid)]
    return TellFactsResponse(facts[fid])



def tell_facts_server():
    rospy.init_node('tell_facts_srv')
    s = rospy.Service('/tell_facts', TellFacts, handle_facts_req)
    print "Ready to tell facts."
    rospy.spin()


if __name__ == "__main__":
    tell_facts_server()
