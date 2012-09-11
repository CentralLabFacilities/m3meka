#! /usr/bin/python



import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.toolbox_core as m3t
import m3.component_factory as m3f
import m3.kontrol as m3k
import time
import math

class M3Proc:
	def __init__(self):
		self.proxy = m3p.M3RtProxy()
		self.gui = m3g.M3Gui(stride_ms=125)#125
		self.slew_rate = 0.5
		self.num_dof = 7
		self.k = m3k.M3Kontrol()

	def stop(self):		
		self.proxy.stop()
		self.k.stop()
	def start(self):
		self.proxy.start()
			
		bot_name=m3t.get_robot_name()
		if bot_name == "":
			print 'Error: no robot components found:', bot_names
			return
		self.bot=m3.humanoid.M3Humanoid(bot_name)	
		arm_names = self.bot.get_available_chains()	
		arm_names = [x for x in arm_names if x.find('arm')!=-1]
		if len(arm_names)==0:
			print 'No arms found'
			return
		if len(arm_names)==1:
			self.arm_name=arm_names[0]
		else:
			self.arm_name = m3t.user_select_components_interactive(arm_names,single=True)[0]

		# ####### Setup Proxy #############
		self.proxy.subscribe_status(self.bot)
		self.proxy.publish_command(self.bot)
		self.proxy.make_operational_all()
		self.bot.set_motor_power_on()
		self.ndof=self.bot.get_num_dof(self.arm_name)
		
		self.theta_curr = [0.0]*self.ndof
		
		zlift_shm_names=proxy.get_available_components('m3joint_zlift_shm')
		if len(zlift_shm_names) > 0:
		  proxy.make_safe_operational(zlift_shm_names[0])

		omnibase_shm_names=proxy.get_available_components('m3omnibase_shm')
		if len(omnibase_shm_names) > 0:
		  proxy.make_safe_operational(omnibase_shm_names[0])

		humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
		if len(humanoid_shm_names) > 0:
		  proxy.make_safe_operational(humanoid_shm_names[0])
		
		
		print 'WARNING: Before switching arm from mode OFF move a slider to initialize positions.'
		
		#Create gui
		self.mode=[0]		
		self.theta_desire_a=[0]*self.num_dof		
		
		self.status_dict=self.proxy.get_status_dict()
		self.param_dict=self.proxy.get_param_dict()
#		self.gui.add('M3GuiModes',  'Mode',      (self,'mode'),range(1),[['Off','Current','Torque','Torque_GC','Theta','Theta_IMP','Jnt_Theta','No Brake'],1],m3g.M3GuiWrite)
		self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=1)
#		self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=1)
				
		self.gui.start(self.step)
		
		

	def step(self):		
		theta_des = [0.0]*self.num_dof
		for i in range(self.num_dof):
			theta_des[i] = (float(self.k.get_slider(i))) / 127.0			
		
		self.slew_rate = (float(self.k.get_slider(self.num_dof))) / 127.0		
	
		self.status_dict=self.proxy.get_status_dict()
		#self.proxy.set_param_from_dict(self.param_dict)		
		
		self.bot.set_mode_theta_gc(self.arm_name)
		#self.bot.set_theta_deg(self.arm_name,self.poses['zero'][self.arm_name])
		self.bot.set_theta_proportion(theta_des)			
		self.bot.set_stiffness(self.arm_name,self.get_stiffness())
		#self.bot.set_thetadot_deg(self.arm_name,[20.0]*self.ndof)
		self.bot.set_slew_rate_proportion([self.slew_rate]*self.num_dof)
		#self.bot.set_slew_rate_proportion(self.arm_name,[1.0]*self.ndof)
		
		
		#self.bot.set_theta_proportion(theta_des)			
		#self.bot.set_mode([self.mode[0]]*self.num_dof)

		#self.bot.set_slew_rate_proportion([self.slew_rate]*self.num_dof)
		self.proxy.step()
		
		
		
if __name__ == '__main__':
	t=M3Proc()
	try:
		t.start()
	except (KeyboardInterrupt,EOFError):
		pass
	t.stop()




