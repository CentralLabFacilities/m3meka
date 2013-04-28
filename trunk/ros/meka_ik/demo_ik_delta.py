#! /usr/bin/python
# -*- coding: utf-8 -*-


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
import roslib; roslib.load_manifest('meka_ik')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from meka_ik.srv import *
from PyKDL import *
from m3.ik_axis import M3IKAxis
import numpy as nu

'''def add_two_ints_client():
    rospy.wait_for_service('meka_ik')
    try:
	tmp = MekaIK()
        add_two_ints = rospy.ServiceProxy('meka_ik', MekaIK)
        resp1 = add_two_ints('right_arm',[0]*3,[0]*3,[0]*7)
        print resp1.success, resp1.angles_solution
        #return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e'''
        
        

def get_pose(bot,arm_name):
    select=[]
    #bot.set_slew_rate_proportion(arm_name, [1.0]*7)
    
    
    q = []
    if arm_name == 'right_arm':
	center = [.338,-.223,.190]			
    else:
	center = [.338,.223,.190]	
	
	
    success = bot.get_tool_position_rpy_2_theta_deg(arm_name, center,[-90,0,-90],q)
    bot.set_mode_theta_gc(arm_name)
    bot.set_theta_deg(arm_name, q)
    proxy.step
    time.sleep(0.3)
    
    
    
    while True:
	print '--------------------------'
	print 'Pose arm'
	print 'Hit enter to set basis pose'
	print '--------------------------'
	os.system("stty raw")
	r = sys.stdin.read(1)
	os.system("stty sane")
	if r=='\r':
	    proxy.step()
	    print 'Basis pose of',bot.get_theta_deg(arm_name)
	    return bot.get_theta_deg(arm_name)

# ######################################################

class ik_thread(Thread):
    def __init__ (self,proxy,bot,step_delta,arm_name,pub, viz):
	Thread.__init__(self)
	self.bot=bot
	self.proxy=proxy
	self.verror=0
	self.aerror=0
	self.delta_done=False
	self.delta=nu.zeros(3) 
	self.target_pos=nu.zeros(3)
	self.pub = pub
	self.target_pos_start = bot.get_tool_position(arm_name)
	self.target_rpy = bot.get_tool_roll_pitch_yaw_deg(arm_name)
	self.step_delta=step_delta
	self.viz = viz
	self.update=True
    def set_delta(self,x):
	self.delta=nu.array(x)
	self.update=True
    def run(self):
	while not self.delta_done:	    
	    self.proxy.step()	 
	    if self.update:
		self.update=False
		self.target_pos=self.target_pos_start+self.delta
		qdes=[]
		success = self.bot.get_tool_position_rpy_2_theta_deg(arm_name, self.target_pos[:], self.target_rpy[:], qdes)
		#success = False
		if success:
		    self.bot.set_theta_deg(arm_name,qdes)
		self.aerror=nu.sqrt(sum((self.target_pos-bot.get_tool_position(arm_name))**2))
		self.bot.set_slew_rate_proportion(arm_name, [0.4]*7)
		self.proxy.step()
		if not self.viz == None:
		    self.viz.step()
	    time.sleep(0.1)
	    
# ######################################################

def run_ik(proxy,bot, step_delta, arm_name,pub, viz):
    pose=get_pose(proxy,bot, arm_name,viz)
    bot.set_mode_theta_gc(arm_name)
    bot.set_stiffness(arm_name, [stiffness]*bot.get_num_dof(arm_name))    
    bot.set_theta_deg(arm_name, pose)
    #bot.set_theta_deg([0,0,0],[4,5,6]) #No roll/pitch/yaw of wrist
    proxy.step()    
    t=ik_thread(proxy,bot,step_delta, arm_name,pub, viz)
    t.start()
    d=[0,0,0]
    while 1:
        print '-----------------------'
	print 'Delta: ',t.delta
	print '-----------------------'
	print 'q: quit'
	print '1: x+'
	print '2: x-'
	print '3: y+'
	print '4: y-'
	print '5: z+'
	print '6: z-'
	print 'space: step'
	k=m3t.get_keystroke()
	if k=='q':
	    t.delta_done=True
	    return
	if k=='1':
	    d[0]=d[0]+step_delta
	if k=='2':
	    d[0]=d[0]-step_delta
	if k=='3':
	    d[1]=d[1]+step_delta
	if k=='4':
	    d[1]=d[1]-step_delta
	if k=='5':
	    d[2]=d[2]+step_delta
	if k=='6':
	    d[2]=d[2]-step_delta
	t.set_delta(d)	
	print
	print 'Error: ',t.aerror
	print 'Target: ',t.target_pos

def humanoid_state_callback(data):
    
	
stiffness=0.5
step_delta=.002 #meters

print 'Select arm:'		
arm_names = ['right_arm', 'left_arm']		
arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

global right_arm_jnt_angles
global left_arm_jnt_angles

right_arm_jnt_angles = [0.0]*7
left_arm_jnt_angles = [0.0]*7

rospy.init_node('demo_ik', anonymous=True)
rospy.Subscriber("humanoid_state", JointState, humanoid_state_callback)

while True:

    print '--------------'
    print 's: set stiffness (Current',stiffness,')'
    print 'd: set step delta (Current',step_delta,'(m))'
    print 'e: execute ijkt controller'
    print 'q: quit'
    print '--------------'
    print
    k=m3t.get_keystroke()
    if k=='q':
	break
    if k=='s':
	print 'Enter stiffness (0-1.0) [',stiffness,']'
	stiffness=max(0,min(1.0,m3t.get_float(stiffness)))
    if k=='d':
	print 'Enter step delta (m) [',step_delta,']'
	step_delta=max(0,min(.25,m3t.get_float(step_delta)))
    if k=='e':
	run_ik(proxy,bot, step_delta,arm_name,pub,viz)


	
    