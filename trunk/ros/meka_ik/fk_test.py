#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.


import time
import os
import roslib; roslib.load_manifest('meka_ik')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from meka_ik.srv import *
from PyKDL import *
from m3.ik_axis import M3IKAxis
import numpy as nu

def fk_client():
    rospy.wait_for_service('meka_fk')
    try:	
        meka_fk = rospy.ServiceProxy('meka_fk', MekaFK)
        resp1 = meka_fk('right_arm',[0.0]*7)
        print 'end_pos:', resp1.end_position
        print 'end rpy:', resp1.end_rpy
        #return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    fk_client()
