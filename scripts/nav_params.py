#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig
import copy

""" NAVIGATION CONSTANTS """

# threshold for being at target
POS_EPS = 0.02
THETA_EPS = 0.05

""" SHARED NAVIGATION PARAMETERS """

class _NavParams():

    def __init__(self):
        self._client = None
        self._params = None

    def _initialize(self):
        self._params = {}
        self._client = Client("nav_params", config_callback=self._callback)
        self._params = copy.deepcopy(self._client.get_configuration())

    def _callback(self, configs):
        self._params.update(configs)

    def _update(self, p, v):
        if not self._client:
            self._initialize()
        self._client.update_configuration({p:v})
        self._params.update({p:v})

    def _get(self, p):
        if not self._client:
            self._initialize()
        return self._params[p]

    @classmethod
    def dot(cls, params):
        for p in params:
            setattr(cls, p, property(fget=lambda s,p=p:s._get(p), fset=lambda s,v,p=p:s._update(p, v)))

if __name__ == "__main__":
    rospy.init_node("nav_params", anonymous = False)
    srv = Server(NavigatorConfig, lambda x,y:x)
    rospy.spin()
else:
    nav_params = _NavParams()
    _NavParams.dot([x['name'] for x in NavigatorConfig.config_description['parameters']])