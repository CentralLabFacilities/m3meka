#M3 -- Meka Robotics Robot Components
#Copyright (C) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import time
import os
import roslib; roslib.load_manifest('m3_defs_ros')
import rospy
from sensor_msgs.msg import JointState
from roslib.msg import Header
import subprocess
        
class M3Viz:
    # ToDo: pass in r_hand_ua component to publish joints
    def __init__ (self,proxy,bot,r_hand_ua_num=None,stride_ms=100):
	self.p = subprocess.Popen(['roslaunch', 'm3_defs_ros', 'm3_launch.launch'])
        rospy.init_node("joint_state_publisher")
        pub = rospy.Publisher("/joint_states", JointState)
        time.sleep(4.0)                    
	self.bot=bot
	self.proxy=proxy	
	self.pub = pub
        self.sleep_time = stride_ms/1000.0
	self.joints = []
        self.positions = []
	self.chain_names = self.bot.get_available_chains()        
        self.use_sim = False
        self.r_hand_ua_present = False
        
        for chain in self.chain_names:
                self.positions += list(self.bot.get_theta_rad(chain))
                self.joints += self.bot.get_joint_names(chain)
        self.ndof_hand_ua = 12
        
        if not r_hand_ua_num is None:
            for i in range(self.ndof_hand_ua):
                self.positions.append(0.0)
                self.joints.append('m3joint_ua_mh'+str(r_hand_ua_num)+'_j'+str(i))
            self.r_hand_ua_present = True

    def step(self):
        self.positions = []
	for chain in self.chain_names:
	    if not self.use_sim:
		self.positions += list(self.bot.get_theta_rad(chain))
                #print self.positions
	    else:
		self.positions += list(self.bot.get_theta_sim_rad(chain))
        if self.r_hand_ua_present:
            self.positions += list([0.0]*self.ndof_hand_ua)
            
	if self.pub != None:
	    if not rospy.is_shutdown():
		    header = Header(0,rospy.Time.now(),'0')
		    self.pub.publish(JointState(header, self.joints, self.positions, [0]*len(self.positions), [0]*len(self.positions)))	    
		    return True
	return False

    def turn_sim_on(self):
        self.use_sim = True
    
    def turn_sim_off(self):
        self.use_sim = False
            
    def stop(self):        
        os.system("pkill -P " + str(self.p.pid))
	os.kill(self.p.pid,9)