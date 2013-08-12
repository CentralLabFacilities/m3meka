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


import m3.rt_proxy as m3p
import m3.pwr as m3rt
import m3.pwr_ec as m3e
import m3.toolbox as m3t
import m3.omnibase as m3o
import time

proxy = m3p.M3RtProxy()
proxy.start()
pwr_name=proxy.get_available_components('m3pwr')
if len(pwr_name)>1:
            pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)

pwr=m3rt.M3Pwr(pwr_name[0])

proxy.subscribe_status(pwr)

base_name=proxy.get_available_components('m3omnibase')
if len(base_name)!=1:
            print 'Invalid number of base components available'
            proxy.stop()
            exit()
omni=m3o.M3OmniBase(base_name[0])
proxy.subscribe_status(omni)

scope_steer = m3t.M3ScopeN(xwidth=100,n=5,title='Steer and Total Currents')
scope_roll = m3t.M3ScopeN(xwidth=100,n=4,title='Roll Currents')


try:
       ts=time.time()
       while True:
            proxy.step()                        
            motor_current = omni.get_motor_torques()
            roll_total_current = []
            str_current = []
            for i in range(4):
                        roll_current.append(motor_current[i*2])
                        str_current.append(motor_current[i*2+1])
            str_current.append(pwr.get_bus_current_mA())
            scope_steer.plot(str_current)
            scope_roll.plot(roll_current)
            #print 'Time: ',60.0-(time.time()-ts),field,':',v
          
            time.sleep(0.1)
            if False:
                        if time.time()-ts>60.0:
                                    print 'Continue [y]?'
                                    if m3t.get_yes_no('y'):
                                           ts=time.time()
                                    else:
                                           break
except (KeyboardInterrupt,EOFError):
       pass
proxy.stop(force_safeop=False) #allow other clients to continue running
