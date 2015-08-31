#!/usr/bin/env python

import rospy
import random

from urllib2 import Request, urlopen, unquote
from facts.srv import *

facts = [ 'The first tanks were built in Lincoln.', 
          'The earliest origins of Lincoln can be traced to the remains of an Iron Age settlement of round wooden dwellings.',
          'The robot Bob tells jocks but they are not that funny.',
          'Lincoln is the only place in the world where you can find an original copy of Magna Carta together with the Charter of the Forest.',
          'George Boole was born in Lincolnshire?',
          'During the 13th century, Lincoln was the third largest city in England',
          'Isaac Newton was born in Lincolnshire?',
          'There are three radio stations in Lincolnshire',
          'In the UK it was illegal to eat mince pies on Christmas Day',
          'Dying is illegal in the Houses of Parliaments',
          'The elephant is the only mammal which can not jump',
          'It is physically impüossible for pigs to look up into the sky',
          "A duck's quack doesn't echo, and no one knows why",
          'Every year about 98%% of atoms in your body are replaced',
          'The is a city called Rome on every continent' ]

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
