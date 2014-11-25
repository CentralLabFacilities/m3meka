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


proxy = m3p.M3RtProxy()
proxy.start()
pwr_name=proxy.get_available_components('m3pwr')
if len(pwr_name)>1:
            pwr_name=m3t.user_select_components_interactive(pwr_name,single=True)
pwr_ec_name=pwr_name[0].replace('m3pwr','m3pwr_ec')
comp=m3rt.M3Pwr(pwr_name[0])
comp_ec=m3e.M3PwrEc(pwr_ec_name)
proxy.subscribe_status(comp)
proxy.publish_command(comp) 
proxy.subscribe_status(comp_ec)
proxy.make_operational(pwr_name[0])
proxy.make_operational(pwr_ec_name)
while True:
            proxy.step()
            print '--------------'
            print 'Hit any key'
            print 'q: quit'
            print 'o: pwr on'
            print 'f: pwr off'
            print '--------------'
            print
            k=m3t.get_keystroke()
            if k=='q':
                        break
            if k=='o':
                        comp.set_motor_power_on()
            if k=='f':
                        comp.set_motor_power_off()
            print '****************'
            print comp.status
            print 
            print comp_ec.status
            print '****************'
proxy.stop()
