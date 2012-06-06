#!/usr/bin/python


import m3.toolbox as m3t
import m3.actuator_ec as m3aec
import m3.actuator as m3a
import m3.ctrl_simple as m3cs
import m3.pwr as m3power


class M3Tuning:
  
	def __init__(self):
		self.comps = {'act': {'name': 'm3dev_actuator_', 'type':m3a.M3RwActuator, 'child_name':'act_ec_component' }, 
		'act_ec': {'name': 'm3dev_actuator_ec_', 'type':m3aec.M3RwActuatorEc, 'child_name':None},
		'ctrl': {'name': 'm3dev_ctrl_simple_', 'type':m3cs.M3RwCtrlSimple, 'child_name':'act_component'},
		'pwr': {'name': 'm3rw_pwr', 'type': m3power.M3Pwr}}
#		'arm' : {'name': 'm3rw_arm_ra0', 'type': m3arm.M3RwArm, 'child_name':'chain_component'},
#		'chain': {'name': 'm3rw_joint_chain', 'type': m3jc.M3RwJointChain, 'child_name':'joint_components'},
#		'joint': {'name': 'm3rw_joint', 'type': m3j.M3RwJoint, 'child_name':'control_component'},
#		'pwr': {'name': 'm3rw_pwr_ra0', 'type': m3power.M3RwPwr}}

	def get_component(self,name):
		if name[-3:-1] == '_j':
			self.comp_name = name
		else:
			cnames = self.proxy.get_available_components(name)
			if len(cnames)==0:
				print 'No ' + name + ' components found. Exiting...'
				exit()	
			self.comp_name = m3t.user_select_components_interactive(cnames,single=True)[0]
		
		self.get_children_components(self.comp_name)
		
	def get_all_components(self,name):
		self.cnames = self.proxy.get_available_components(name)
		if len(self.cnames)==0:
			print 'No ' + name + ' components found. Exiting...'
			raise Exception('no available components')
		
		for cname in self.cnames:
			self.child_names[cname] = self.get_all_children_components(cname)
		
		
	# component list would be 'act','act_ec','ctrl','pwr'
	def start_all_components(self,component_list,operational_list):
		for cname in self.cnames:
		# accomplishes this: self.act=m3s.M3Actuator(self.comp_name)
			self.act_list[cname] = m3s.M3Actuator(cname)
			setattr(self, k, v['type'](v['name']) )
			self.comps[k]['comp'] = getattr(self,k)
			self.proxy.subscribe_status(cname)
			self.proxy.publish_command(cname) 
			self.proxy.publish_param(cname) 
			if operational_list is None or k in operational_list:
				self.proxy.make_operational(v['name'])
		
		if 'pwr' in component_list:
			print 'making power operational'
			self.proxy.make_operational(self.pwr.get_actuator_ec_name())
			self.pwr.set_motor_power_on()	
		self.proxy.step()
		
	def start_components(self,component_list,operational_list):
		for k in component_list:
			v = self.comps[k]
		# accomplishes this: self.act=m3s.M3Actuator(self.comp_name)
			setattr(self, k, v['type'](v['name']) )
			self.comps[k]['comp'] = getattr(self,k)
			self.proxy.subscribe_status(getattr(self,k))
			self.proxy.publish_command(getattr(self,k)) 
			self.proxy.publish_param(getattr(self,k)) 
			if operational_list is None or k in operational_list:
				self.proxy.make_operational(v['name'])
		
		if 'pwr' in component_list:
			print 'making power operational'
			self.proxy.make_operational(self.pwr.get_actuator_ec_name())
			self.pwr.set_mode_on()	
		self.proxy.step()

	def get_children_components(self,comp_name):
		self.joint_suffix = comp_name[-2:]
		for k in ['act','act_ec','ctrl']:
			self.comps[k]['name'] = self.comps[k]['name'] + self.joint_suffix
			
	def get_all_children_components():
		child_dict = {}
		self.param_dict=self.proxy.get_param_dict()
		if re.match(r"m3rw_actuator_ra0_j\d",comp_name):
			child_dict['act_ec'] = self.param_dict[comp_name]['act_ec_component']
			  