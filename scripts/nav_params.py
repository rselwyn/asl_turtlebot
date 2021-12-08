import rospy
from dynamic_reconfigure.client import Client
from asl_turtlebot.cfg import NavigatorConfig


""" NAVIGATION CONSTANTS """

# threshold for being at target
POS_EPS = 0.02
THETA_EPS = 0.05

""" SHARED NAVIGATION PARAMETERS """

_defs = [
    "localizer", # "odom" or "map"
    "v_max", # maximum velocity
    "om_max", # maximum angular velocity
]

class _NavParams():

    def __init__(self):
        self._params = {}
        self._client = Client(NavigatorConfig, self._callback)

    def _callback(self, configs):
        self._params.update({k:v for k,v in zip(self._params.keys(), configs)})

    def _update(self, p, v):
        self._client().

    @classmethod
    def dot(cls, params):
        for p in params:
            setattr(cls, p, property(fget=lambda s,p=p:s._params[p], fset=lambda s,v,p=p:rospy.set_param(p, v)))
        return _NavParams()

nav_params = _NavParams()
_NavParams.dot(nav_params._params.keys())