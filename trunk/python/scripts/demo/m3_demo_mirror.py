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
import m3.rt_proxy as m3p
import m3.toolbox as m3t
from m3.unit_conversion import *
import m3.component_factory as m3f
import numpy as nu
import m3.humanoid 
import math

# ######################################################	
proxy = m3p.M3RtProxy()
proxy.start()
bot_name=m3t.get_robot_name()
if bot_name == "":
	print 'Error: no robot components found:', bot_name
bot=m3f.create_component(bot_name)
proxy.publish_param(bot) #allow to set payload
proxy.subscribe_status(bot)
proxy.publish_command(bot)
proxy.make_operational_all()
bot.set_motor_power_on()
proxy.step()

humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
if len(humanoid_shm_names) > 0:
  proxy.make_safe_operational(humanoid_shm_names[0])

#chains=bot.get_available_chains()
#print 'Select chain'
#chains=m3t.user_select_components_interactive(chains,single=True)

stiffness=1.0
#print 'Enter stiffness (0-1.0) [',stiffness,']'
#stiffness=max(0,min(1.0,m3t.get_float(stiffness)))

target_arm = 'right_arm'
input_arm = 'left_arm'

ndof=7

bot.set_mode_theta_gc(target_arm)
bot.set_stiffness(target_arm,[stiffness]*ndof)	
bot.set_slew_rate_proportion(target_arm,[1.0]*ndof)

bot.set_mode_pose(input_arm)
bot.set_stiffness(input_arm,[stiffness]*ndof)	
bot.set_slew_rate_proportion(input_arm,[1.0]*ndof)

target_pose = bot.get_theta_deg(input_arm)
bot.set_theta_deg(target_arm, target_pose)

bot.set_theta_deg('head', [0.0,0.0])
bot.set_mode_theta('head')
bot.set_slew_rate_proportion('head',[1.0, 1.0])

try:
	while True:
		proxy.step()
		target_pose = bot.get_theta_deg(input_arm)
		target_pose[1] *= -1
		target_pose[2] *= -1
		target_pose[4] *= -1
		target_pose[6] *= -1
		
		bot.set_theta_deg(target_arm, target_pose)
		tool_pos = bot.get_tool_position(target_arm)
		tool_pos[2] -= 0.2
		#head_pos = bot.get_tool_position('head')
		#print 'pos:', tool_pos
		#print 'head:', head_pos
		yaw = rad2deg(math.atan2(tool_pos[1], tool_pos[0]))
		pitch = rad2deg(math.atan2(tool_pos[2], tool_pos[0]))
		#print 'yaw angle:',  yaw
		#print 'pitch angle:', pitch
		bot.set_theta_deg('head', [pitch, yaw])
		'''for c in chains:
			print '---------------------------------------------'
			print 'Chain: ',c
			print 'Tool Position: (m)',bot.get_tool_position(c)
			print 'Theta (Deg): ',bot.get_theta_deg(c)
			print 'Tool Velocity (m/S)',bot.get_tool_velocity(c)'''
		#time.sleep(0.1)
except (KeyboardInterrupt,EOFError):
	proxy.stop()
