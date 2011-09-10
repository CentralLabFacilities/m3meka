#! /usr/bin/python
# -*- coding: utf-8 -*-

import m3.toolbox as m3t
import m3.monitor
import m3.arm
import m3.torso
import m3.head
import m3.hand
#import m3.hand_ua
import m3.gripper
import m3.actuator_ec
import m3.actuator
import m3.joint
import m3.joint_slave
import m3.joint_zlift
import m3.pwr
import m3.pwr_ec
import m3.joint
import m3.loadx1
import m3.loadx1_ec
import m3.loadx6
import m3.loadx6_ec
import m3.humanoid
import m3.dynamatics
import m3.omnibase
#import m3.tactile_pps22_ec
import m3.ledx2xn_ec
import m3.led_matrix_ec
import m3.head_s2csp_ctrl
#import m3skin.taxel_array_ec

component_map={
    'm3monitor': m3.monitor.M3Monitor,
    'm3arm': m3.arm.M3Arm,
    'm3torso': m3.torso.M3Torso,
    'm3head': m3.head.M3Head,
    'm3hand': m3.hand.M3Hand,
    #'m3hand_ua': m3.hand_ua.M3HandUA,
    'm3gripper': m3.gripper.M3Gripper,
    'm3actuator_ec':m3.actuator_ec.M3ActuatorEc,
    'm3actuator':m3.actuator.M3Actuator,
    'm3actuator_virtual':m3.actuator.M3Actuator,
    'm3joint':m3.joint.M3Joint,
    'm3joint_slave':m3.joint_slave.M3JointSlave,
    'm3pwr':m3.pwr.M3Pwr,
    'm3pwr_ec':m3.pwr_ec.M3PwrEc,
    'm3pwr_virtual':m3.pwr.M3Pwr,
    'm3loadx1':m3.loadx1.M3LoadX1,
    'm3loadx1_ec':m3.loadx1_ec.M3LoadX1Ec,
    'm3loadx6':m3.loadx6.M3LoadX6,
    'm3loadx6_ec':m3.loadx6_ec.M3LoadX6Ec,    
    'm3humanoid':m3.humanoid.M3Humanoid,
    'm3dynamatics':m3.dynamatics.M3Dynamatics,
    'm3head_s2csp_ctrl':m3.head_s2csp_ctrl.M3HeadS2CSPCtrl,
    'm3joint_zlift':m3.joint_zlift.M3JointZLift,
    'm3omnibase':m3.omnibase.M3OmniBase,
    'm3ledx2xn_ec': m3.ledx2xn_ec.M3LedX2XNEc,
    'm3led_matrix_ec': m3.led_matrix_ec.M3LedMatrixEc,
    #'m3taxel_array_ec': m3skin.taxel_array_ec.M3TaxelArrayEc,
    #'m3tactile_pps22_ec':m3.tactile_pps22_ec.M3TactilePPS22Ec,
    
    }

def create_component(name):
    """This is a useful utility for creating components based
    on the name only. The m3_config.yml file maps component names
    to types. This is used to figure out the type and instantiate
    a new component class"""
    ttype=m3t.get_component_config_type(name)
    if ttype=='':
        print 'Component Factory type not found for component',name
        return None
    if not component_map.has_key(ttype):
        print 'Component Factory type ',ttype, 'not found in component_map for',name
        return None
    return component_map[ttype](name)



