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
import roslib; roslib.load_manifest('shm_humanoid_controller')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from m3ctrl_msgs.msg import *
from PyKDL import *
import numpy as nu
import m3.joint_mode_ros_pb2 as mab
import m3.smoothing_mode_pb2 as mas
import m3.unit_conversion as m3u
import m3.toolbox as m3t
import string

class M3Chain:
    (RIGHT_ARM, LEFT_ARM, TORSO, HEAD) = range(4)


rospy.init_node('demo_head_s3', anonymous=True)

global humanoid_pub
humanoid_pub = rospy.Publisher('humanoid_command', M3JointCmd)


#rospy.init_node('demo_head_s3', anonymous=True)

cmd = M3JointCmd()
cmd.chain = [0]*2
cmd.control_mode = [0]*2
cmd.smoothing_mode = [0]*2

for i in range(2):  
  cmd.chain[i] = (int(M3Chain.HEAD))  
  cmd.stiffness.append(1.0)
  cmd.position.append(0.0)
  cmd.velocity.append(1.0)
  cmd.effort.append(0.0)
  cmd.control_mode[i] = (int(mab.JOINT_MODE_ROS_THETA))
#  cmd.smoothing_mode[i] = (int(mas.SMOOTHING_MODE_MIN_JERK))
  cmd.smoothing_mode[i] = (int(mas.SMOOTHING_MODE_SLEW))
  cmd.chain_idx.append(i)


try:
      cmd.header = Header(0,rospy.Time.now(),'0')
      humanoid_pub.publish(cmd)

      rospy.sleep(3.0)
      
      while not rospy.is_shutdown():        
	
	  print 'Moving to Position 1'
	  cmd.position[0] = -0.17
	  cmd.position[1] = 0.78
	  rospy.sleep(3.0)
	  
	  cmd.header = Header(0,rospy.Time.now(),'0')
	  humanoid_pub.publish(cmd)
	  
	  print 'Moving to Position 2'
	  cmd.position[0] = 0.05
	  cmd.position[1] = -0.78
	  rospy.sleep(3.0)
	  
	  cmd.header = Header(0,rospy.Time.now(),'0')
	  humanoid_pub.publish(cmd)
	  
	  print 'Moving to Position 3'
	  cmd.position[0] = 0.0
	  cmd.position[1] = 0.0
	  rospy.sleep(3.0)
	  
	  cmd.header = Header(0,rospy.Time.now(),'0')
	  humanoid_pub.publish(cmd)
	  

	  print 'Repeating'
	  rospy.sleep(0.5)

except (KeyboardInterrupt,EOFError):
      pass

    