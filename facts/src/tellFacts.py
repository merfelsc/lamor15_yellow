#!/usr/bin/env python

import rospy
import random

from urllib2 import Request, urlopen, unquote
from facts.srv import *

facts = [ 'The first tanks were built in Lincoln.', 
          'The earliest origins of Lincoln can be traced to the remains of an Iron Age settlement of round wooden dwellings.',
          'The robot Bob tells jocks but they are not that funny.',
          'Lincoln is the only place in the world where you can find an original copy of Magna Carta together with the Charter of the Forest, issued in 1217 to amplify the document and one of only two surviving copies.',
          'Did you know that George Boole was born in Lincolnshire?' ]

history = {}

def handle_facts_req(req):
    pid = req.person

    print 'Request for person ', pid

    # just try to fetch a chuck noris joke if offline this just throws an exception and normal
    # handling will take over
    if pid in history and len(history[pid]) == 0:
        # never eval arbitrary website content ;)
        try:
            joke = unquote(eval(urlopen('http://api.icndb.com/jokes/random', None, 1).read())['value']['joke'])
            joke = joke.replace('&quot;', '"')
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
