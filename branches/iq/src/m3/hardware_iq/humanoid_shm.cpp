/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <humanoid_shm.h>
#include "m3rt/base/component_factory.h"


namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3HumanoidShm::GetBaseStatus()
{
	return status.mutable_base();
}

void  M3HumanoidShm::Startup()
{  
  M3CompShm::Startup();
  
  sds_status_size = sizeof(M3HumanoidShmSdsStatus);
  sds_cmd_size = sizeof(M3HumanoidShmSdsCommand);  
  
  memset(&status_to_sds, 0, sds_status_size);
  
}

void M3HumanoidShm::ResetCommandSds(unsigned char * sds)
{
  
  memset(sds,0,sizeof(M3HumanoidShmSdsCommand));
  
}


size_t M3HumanoidShm::GetStatusSdsSize()
{
	return sds_status_size;
}

size_t M3HumanoidShm::GetCommandSdsSize()
{
	return sds_cmd_size;
}

void M3HumanoidShm::SetCommandFromSds(unsigned char * data)
{
  
  M3HumanoidShmSdsCommand * sds = (M3HumanoidShmSdsCommand *) data;
    request_command();
   memcpy(&command_from_sds, sds, GetCommandSdsSize()); 
    release_command();    
    
    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS(dt) > (timeout*1000);    
       
  if (bot != NULL)
  {
    if (shm_timeout)
      bot->SetMotorPowerOff();
    else
      bot->SetMotorPowerOn();
    
    for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
    {       
      if (shm_timeout)
      {	
	bot->SetModeOff(RIGHT_ARM, i);
      } else{
	bot->SetThetaDeg(RIGHT_ARM, i, command_from_sds.right_arm.q_desired[i]);
	bot->SetSlewRate(RIGHT_ARM, i, command_from_sds.right_arm.slew_rate_q_desired[i]);
	bot->SetTorque_mNm(RIGHT_ARM, i, command_from_sds.right_arm.tq_desired[i]);
	bot->SetStiffness(RIGHT_ARM, i, command_from_sds.right_arm.q_stiffness[i]);
	((M3HumanoidCommand*)bot->GetCommand())->mutable_right_arm()->set_ctrl_mode(i, command_from_sds.right_arm.ctrl_mode[i]);
      }
    }
  }
  
  if (right_hand)
  { 
      for (int i = 0; i < right_hand->GetNumDof(); i++)
      {
	if (shm_timeout)
	{	
	  ((M3JointArrayCommand*)right_hand->GetCommand())->set_ctrl_mode(i, JOINT_ARRAY_MODE_OFF);
	} else{	  	  
	  ((M3JointArrayCommand*)right_hand->GetCommand())->set_ctrl_mode(i, command_from_sds.right_hand.ctrl_mode[i]);      
	  ((M3JointArrayCommand*)right_hand->GetCommand())->set_q_desired(i, command_from_sds.right_hand.q_desired[i]);
	  ((M3JointArrayCommand*)right_hand->GetCommand())->set_q_slew_rate(i, command_from_sds.right_hand.slew_rate_q_desired[i]);    
	  ((M3JointArrayCommand*)right_hand->GetCommand())->set_tq_desired(i, command_from_sds.right_hand.tq_desired[i]);      
	}
      }	      
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3HumanoidShm::SetSdsFromStatus(unsigned char * data)
{  
  status_to_sds.timestamp = GetBaseStatus()->timestamp(); 
  
  if (bot)
  {
    for (int i = 0; i < bot->GetNdof(RIGHT_ARM); i++)
    {          
      status_to_sds.right_arm.theta[i] = bot->GetThetaDeg(RIGHT_ARM,i);
      status_to_sds.right_arm.thetadot[i] = bot->GetThetaDotDeg(RIGHT_ARM,i);
      status_to_sds.right_arm.torque[i] = bot->GetTorque_mNm(RIGHT_ARM,i);      
    }
  }
    
  if (right_hand)
  { 
      for (int i = 0; i < right_hand->GetNumDof(); i++)
      {	
	status_to_sds.right_hand.theta[i] = right_hand->GetThetaDeg(i);
	status_to_sds.right_hand.thetadot[i] = right_hand->GetThetaDotDeg(i);
	status_to_sds.right_hand.torque[i] = right_hand->GetTorque(i);
      }
  }
  
  if (right_loadx6)
  {
    for (int i = 0; i < 6; i++)
    {      
      status_to_sds.right_loadx6.wrench[i] = right_loadx6->GetWrench(i);
    }
  }
  
  M3HumanoidShmSdsStatus * sds = (M3HumanoidShmSdsStatus *) data;
  request_status();  
  memcpy(sds, &status_to_sds, GetStatusSdsSize());  
  release_status();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3HumanoidShm::LinkDependentComponents()
{
	tmp_cnt = 0;
	
	if (bot_name.size()!=0)
	{		
		bot=(M3Humanoid*)factory->GetComponent(bot_name);
		if (bot==NULL)
		{
			M3_ERR("M3Humanoid component %s declared for M3BotShm but could not be linked\n",
					bot_name.c_str());
		    //return false;
		}
	}
	
	if (right_hand_name.size()!=0)
	{
		right_hand=(M3Hand*)factory->GetComponent(right_hand_name);//May be null if not on this robot model
		if (right_hand==NULL)
		{
			M3_WARN("M3Hand component %s declared for M3BotShm but could not be linked\n",
					right_hand_name.c_str());
		    //return false;
		}				
	}
	
	if (right_loadx6_name.size()!=0)
	{
		right_loadx6=(M3LoadX6*)factory->GetComponent(right_loadx6_name);
		if (right_loadx6==NULL)
		{
			M3_WARN("M3LoadX6 component %s declared for M3BotShm but could not be linked\n",
					right_loadx6_name.c_str());
		    //return false;
		}				
	}

	if (right_loadx6 || right_hand || bot)
	{	 
	  return true;	  
	} else {	  
	  M3_ERR("Could not link any components for M3BotShm shared memory.\n");
	  return false;
	}
}

bool M3HumanoidShm::ReadConfig(const char * filename)
{	
	if (!M3CompShm::ReadConfig(filename))
		return false;

	YAML::Node doc;	
	GetYamlDoc(filename, doc);	
	
	try{
	  doc["humanoid_component"] >> bot_name;	
	}
	catch(YAML::KeyNotFound& e)
	{
	  bot_name="";
	}
	
	try{
	  doc["right_hand_component"] >> right_hand_name;	
	}
	catch(YAML::KeyNotFound& e)
	{
	  right_hand_name="";
	}
	
	try{
	  doc["right_loadx6_component"] >> right_loadx6_name;	
	}
	catch(YAML::KeyNotFound& e)
	{
	  right_loadx6_name="";
	}
		
	try{
	  doc["startup_motor_pwr_on"]>>startup_motor_pwr_on;
	}
	catch(YAML::KeyNotFound& e)
	{
	  startup_motor_pwr_on=false;
	}
		
	doc["timeout"] >> timeout;	
	
	return true;
}
}
   
