#! /usr/bin/python

#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
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

#
import yaml
import os 
from m3.toolbox import *
import m3.ctrl_simple_pb2
import m3.actuator_pb2  as a
from m3.component import M3Component
from m3.unit_conversion import *

	
class M3CtrlSimple(M3Component):
	"""Calibrated interface for the M3 ctrl simple"""
	def __init__(self,name,type='m3ctrl_simple'):
		M3Component.__init__(self,name,type=type)
		self.status		= m3.ctrl_simple_pb2.M3CtrlSimpleStatus()
		self.command	= m3.ctrl_simple_pb2.M3CtrlSimpleCommand()
		self.param		= m3.ctrl_simple_pb2.M3CtrlSimpleParam()
		self.read_config()

# SETTERS
	# modes
	def set_control_mode(self,m):
		self.command.ctrl_mode = m
		
	def set_traj_mode(self,m):
		self.command.traj_mode = m
	
	def set_mode_off(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_OFF

	def set_mode_current(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_CURRENT
		
	def set_mode_theta(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_THETA

	def set_mode_torque(self):
		self.command.ctrl_mode = m3.ctrl_simple_pb2.CTRL_MODE_TORQUE
		
		
	# commands
	def set_current(self, t):
		self.command.desired_current = t
	def set_theta(self, t):
		self.command.desired_theta = t
	def set_theta_deg(self, t):
		self.command.desired_theta = t*math.pi/180.0
	def set_torque(self, t):
		self.command.desired_torque = t

# GETTERS
	
	# misc	
	def get_timestamp_uS(self):
		return self.status.base.timestamp
	
	def get_flags(self):
		return self.status.flags
	
