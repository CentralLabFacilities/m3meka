#! /usr/bin/python

# MEKA CONFIDENTIAL
# 
# Copyright 2011 
# Meka Robotics LLC
# All Rights Reserved.
# 
# NOTICE:  All information contained herein is, and remains
# the property of Meka Robotics LLC. The intellectual and 
# technical concepts contained herein are proprietary to 
# Meka Robotics LLC and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material
# is strictly forbidden unless prior written permission is obtained
# from Meka Robotics LLC.

#import matplotlib
#matplotlib.use('TkAgg')
import time
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.actuator_pb2 as mec
import m3.actuator as m3s
import m3.component_factory as m3f
import math
import glob 

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)
		self.cnt=0
		self.bias=[]
	def stop(self):
		self.act.set_mode_off()
		self.proxy.step()
		self.proxy.stop()
	def start(self):
		self.proxy.start(start_data_svc=True,start_ros_svc=False)
		self.comp_name='m3lcj_simple_j0'
		
		
		act_names=self.proxy.get_available_components('m3actuator')
		if len(act_names)==0:
			print 'No actuator components available'
			self.proxy.stop()
			exit()
		
		act_name=m3t.user_select_components_interactive(act_names,single=True)[0]
		act_ec_name=m3t.get_actuator_ec_component_name(act_name)
		pwr_name=m3t.get_actuator_ec_pwr_component_name(act_ec_name)
		
		self.act=m3f.create_component(act_name)
		self.act_ec=m3f.create_component(act_ec_name)
		self.pwr=m3f.create_component(pwr_name)
		
		self.proxy.subscribe_status(self.act_ec)
		self.proxy.publish_param(self.act_ec) 
		self.proxy.subscribe_status(self.act)
                self.proxy.publish_command(self.act)
                self.proxy.publish_param(self.act) 
		self.proxy.publish_command(self.pwr)
	    
		

		#Start them all up
		self.proxy.make_operational_all()
		self.pwr.set_motor_power_on()
		
		#Force safe-op of robot, etc are present
		types=['m3humanoid','m3hand','m3gripper']
		for t in types:
			cc=self.proxy.get_available_components(t)
			for ccc in cc:
				self.proxy.make_safe_operational(ccc)
		
		self.proxy.step()

		#Create gui
		self.mode=[0]
		self.traj=[0]
	
		self.pwm_desired=[0]
		self.torque_desired=[0]
		self.i_desired=[0]
	
		self.save=False
		self.save_last=False
		self.do_scope=False
		self.scope=None
		self.status_dict=self.proxy.get_status_dict()
		
		#extract status fields
		self.scope_keys=m3t.get_msg_fields(self.act.status)
		self.scope_keys.sort()
		self.scope_keys=['None']+self.scope_keys
		self.scope_field1=[0]
		self.scope_field2=[0]

		act_tq_max=10000 #stall torque of RE30 60W=1Nm, 100:1
		act_pwm_max=3186
		act_i_max=5000
		
		self.param_dict=self.proxy.get_param_dict()
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(1),[['Off','PWM','Torque','Current',
		                                                                  'TorqueIFF'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Traj',      (self,'traj'),range(1),[['Off','TorqueSquare','TorqueSine',
		                                                                  'JointThetaSquare','JointThetaSine'],1],m3g.M3GuiWrite)
		
		self.gui.add('M3GuiSliders','PWM',  (self,'pwm_desired'),range(1),[-act_pwm_max,act_pwm_max],m3g.M3GuiWrite)		
		self.gui.add('M3GuiSliders','Torque(mNm)',  (self,'torque_desired'),range(1),[-act_tq_max,act_tq_max],m3g.M3GuiWrite)
		self.gui.add('M3GuiSliders','Current(mA)',  (self,'i_desired'),range(1),[-act_i_max,act_i_max],m3g.M3GuiWrite)
		
		self.gui.add('M3GuiToggle', 'Save',      (self,'save'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Scope1',      (self,'scope_field1'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiModes',  'Scope2',      (self,'scope_field2'),range(1),[self.scope_keys,1],m3g.M3GuiWrite)
		self.gui.add('M3GuiToggle', 'Scope',      (self,'do_scope'),[],[['On','Off']],m3g.M3GuiWrite)
		self.gui.start(self.step)

	
	def step(self):
		if self.do_scope and self.scope is None:
			self.scope=m3t.M3Scope2(xwidth=100,yrange=None)
		self.proxy.step()
		self.cnt=self.cnt+1
		self.status_dict=self.proxy.get_status_dict()
		self.proxy.set_param_from_dict(self.param_dict)

		if not self.do_scope:
			print 'Theta (deg)',self.act.get_theta_deg()
			print 'Torque (mNm)',self.act.get_torque_mNm()
		
		if self.do_scope and self.scope is not None:
			f1=self.scope_keys[self.scope_field1[0]]
			f2=self.scope_keys[self.scope_field2[0]]
			x1=x2=None
			if f1!='None' and f1!='base':
				x1=m3t.get_msg_field_value(self.act.status,f1)
				print f1,':',x1
			if f2!='None' and f2!='base':
				x2=m3t.get_msg_field_value(self.act.status,f2)   
				print f2,':',x2
			if x1==None:
				x1=x2
			if x2==None:
				x2=x1
			if x1!=None and x2!=None: #Handle only one value or two
				self.scope.plot(x1,x2)
				print'-----------------'
		
		
		
		self.act.set_traj_mode(self.traj[0])
			
		if self.mode[0]==mec.ACTUATOR_MODE_OFF:
			self.act.set_mode_off()
		if self.mode[0]==mec.ACTUATOR_MODE_PWM:
			self.act.set_mode_pwm()
			self.act.set_pwm(int(self.pwm_desired[0]))
		if self.mode[0]==mec.ACTUATOR_MODE_TORQUE:
			self.act.set_mode_torque()
			self.act.set_torque_mNm(self.torque_desired[0])
		if self.mode[0]==mec.ACTUATOR_MODE_CURRENT:
			self.act.set_mode_current()
			self.act.set_current_mA(self.i_desired[0])
		if self.mode[0]==mec.ACTUATOR_MODE_TORQUE_IFF:
			self.act.set_mode_torque_iff()
			self.act.set_torque_mNm(self.torque_desired[0])
	
		if (self.save and not self.save_last):
			self.act.write_config()
		self.save_last=self.save

if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()



