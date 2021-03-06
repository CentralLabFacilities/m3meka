#! /usr/bin/python

#Copyright  2010, Meka Robotics
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
import numpy as nu
from m3.unit_conversion import *
import m3.component_factory as m3f
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.omnibase as m3o
import m3.toolbox_ros as m3tr
import roslib; roslib.load_manifest('m3_client')
import rospy
from m3_client.msg import M3OmnibaseJoy

max_lin_vel = 0.25  # m/s (0.6)
max_lin_acc = 0.4 # m/s^2  (0.2)  --- 1.0 gives really good performance but saturates motors...
max_rot_vel = 25   # deg/s
max_rot_acc = 60  # 100 for better    # deg/s^2


def main():   
    
    proxy = m3p.M3RtProxy()
    proxy.start()    
    
    base_name=proxy.get_available_components('m3omnibase')
    if len(base_name)!=1:
            print 'Invalid number of base components available'
            proxy.stop()
            exit()
    omni=m3o.M3OmniBase(base_name[0])

    proxy.publish_param(omni) # we need this for calibration
    proxy.subscribe_status(omni)
    proxy.publish_command(omni)
    
    pwr_name=[m3t.get_omnibase_pwr_component_name(base_name[0])]
    #proxy.get_available_components('m3pwr')
    #if len(pwr_name)>1:
            #pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)
    pwr=m3f.create_component(pwr_name[0])
    
    proxy.subscribe_status(pwr)
    proxy.publish_command(pwr) 
    proxy.make_operational(pwr_name[0])
    proxy.step()
    omni.set_mode_off()
    pwr.set_motor_power_on()    
    proxy.make_operational_all()
    
    proxy.step()
    time.sleep(0.5)
    proxy.step()
    omni.calibrate(proxy)
    time.sleep(0.5)
    print "Turn power on to robot and press any key."
    raw_input()      
    omni.set_local_position(0,0,0,proxy)
    omni.set_global_position(0,0,0,proxy)
    omni.set_max_linear_accel(max_lin_acc)
    omni.set_max_linear_velocity(max_lin_vel)
    omni.set_max_rotation_velocity(max_rot_vel)
    omni.set_max_rotation_accel(max_rot_acc)
    
    proxy.step()
    '''p = omni.get_global_position()
    print 'Pos:', p'''
    omni.set_mode_traj_goal()
    omni.set_traj_goal(0, 0, 0)
    
    proxy.step()
    time.sleep(4)
    print 'Bus voltage:',omni.get_bus_voltage()
    print 'Press any key to begin pacing.'
    raw_input()

    try:
        while True:            
            omni.set_traj_goal(2.0, 0, 180)            
            proxy.step()
            time.sleep(0.1)
            proxy.step()
            
            while not omni.is_traj_goal_reached():                
                proxy.step()
                p = omni.get_global_position()
                print 'Pos:', p
                time.sleep(0.1)
           
            omni.set_traj_goal(0, 0, 0)
            proxy.step()
            time.sleep(0.1)
            proxy.step()
            
            while not omni.is_traj_goal_reached():
                proxy.step()
                p = omni.get_global_position()
                print 'Pos:', p
                time.sleep(0.1)                    
    except KeyboardInterrupt:
        pass
    
    
    omni.set_mode_off()
    pwr.set_motor_power_off()
    
    proxy.step()
    proxy.stop()

# allow use as a module or standalone script  
if __name__ == "__main__":  
    main()  

