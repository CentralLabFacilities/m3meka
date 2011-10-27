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

#include "m3/chains/joint_array.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/m3rt_def.h"

namespace m3
{
	
using namespace m3rt;
using namespace std;
using namespace KDL;
using namespace ros;

///////////////////////////////////////////////////////
//Success so long as least one dependent comp. available
bool M3JointArray::LinkDependentComponents()
{	
	int nl=0;
	if (joint_names.size()>0)
		joints.assign(joint_names.size(),(M3Joint*)NULL);
	for (int i=0;i<joint_names.size();i++)
	{
		if(joint_names[i].size())
		{
			joints[i]=(M3Joint*)factory->GetComponent(joint_names[i]);			
			if (joints[i]!=NULL)
				nl++;
		}
		else
			joints[i]=NULL;
	}	
		
	if (nl<=0)
	{
		M3_ERR("M3JointArray %s found %d joints \n",GetName().c_str(),nl);	
		return false;
	}
	if (nl!=ndof)
	{
		M3_ERR("M3JointArray %s found %d joints. Expected %d. Continuing with missing joints. \n",
		       GetName().c_str(),nl,ndof);	
	}
	return true;
}

void M3JointArray::Startup()
{		
	joints.resize(ndof);
	traj_active.assign(ndof,false);
	joint_names.resize(ndof);
	traj_des.resize(ndof);
	theta.resize(ndof);
	thetadot.resize(ndof);
	thetadotdot.resize(ndof);
	torque.resize(ndof);
	tw_torque.resize(ndof);

	for (int i=0;i<ndof;i++)
	{
		if (joint_names[i].size()==0)
		{
			M3_WARN("M3JointArray joint %d not configured properly\n",i);
			SetStateError();
			return;
		}
		command.add_q_desired(0);
		command.add_qdot_desired(0);
		command.add_pwm_desired(0);
		command.add_tq_desired(0);
		command.add_q_stiffness(0);
		command.add_q_slew_rate(0);
		command.add_ctrl_mode(JOINT_ARRAY_MODE_OFF);
		
		status.add_motor_temp(0);
		status.add_amp_temp(0);
		status.add_current(0);
		status.add_torque(0);
		status.add_torquedot(0);		
		status.add_theta(0);
		status.add_thetadot(0);
		status.add_thetadotdot(0);
		status.add_pwm_cmd(0);
		status.add_flags(0);
	}
	status.set_completed_spline_idx(-1);
	
	SetStateSafeOp();

}


void M3JointArray::Shutdown()
{

}


void M3JointArray::StepJointTrajectory()
{
	bool reset=false; //Check for change in active DOF
	//See if active joints has changed
	for (int i=0;i<ndof;i++)
	{
		bool st = command.ctrl_mode(i)==JOINT_ARRAY_MODE_SPLINED_TRAJ;
		bool stg= command.ctrl_mode(i)==JOINT_ARRAY_MODE_SPLINED_TRAJ_GC;
		if ( ( (st||stg) && !traj_active[i]) || ((!st&&!stg) && traj_active[i]))
			reset=true;
		traj_active[i]=(st||stg);
	}
	if (reset)
	{
		traj.Reset(traj_active,ndof);
	}
	//Add vias
	for (int i=0;i<command.vias_size();i++)
		traj.AddVia(command.vias(i));
	command.clear_vias();
	
	for (int i=0;i<ndof;i++)
	{
		theta(i)=status.theta(i);
		thetadot(i)=status.thetadot(i);
	}
	int idx=traj.Step(status.base().timestamp(),theta,thetadot,traj_des);
	status.set_completed_spline_idx(idx);
}


void M3JointArray::StepCommand()
{	
	if (IsStateSafeOp() || IsStateError())
		return;
	
	StepJointTrajectory();

	//Individual controllers
	for (int i=0;i<ndof;i++)
	{

		if (joints[i]!=NULL)
		{		
			joints[i]->SetDesiredThetaDeg(command.q_desired(i));
			joints[i]->SetDesiredThetaDotDeg(command.qdot_desired(i));
			joints[i]->SetDesiredPwm(command.pwm_desired(i));
			joints[i]->SetDesiredTorque(command.tq_desired(i));
			joints[i]->SetDesiredStiffness(command.q_stiffness(i));
			joints[i]->SetSlewRate(command.q_slew_rate(i));

			switch(command.ctrl_mode(i))
			{										
				case JOINT_ARRAY_MODE_OFF:
					joints[i]->SetDesiredControlMode(JOINT_MODE_OFF);
					break;
				case JOINT_ARRAY_MODE_PWM:
					joints[i]->SetDesiredControlMode(JOINT_MODE_PWM);
					break;
				case JOINT_ARRAY_MODE_TORQUE:
					joints[i]->SetDesiredControlMode(JOINT_MODE_TORQUE);
					break;
				case JOINT_ARRAY_MODE_THETA:

					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA);
					break;
				case JOINT_ARRAY_MODE_TORQUE_GC:
					joints[i]->SetDesiredControlMode(JOINT_MODE_TORQUE_GC);
					break;
				case JOINT_ARRAY_MODE_THETA_GC:					
					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA_GC);
					break;
				case JOINT_ARRAY_MODE_THETA_MJ:
					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA_MJ);
					break;
				case JOINT_ARRAY_MODE_THETA_GC_MJ:
					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA_GC_MJ);
					break;
				case JOINT_ARRAY_MODE_SPLINED_TRAJ_GC:
				{
					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA_GC);	
					joints[i]->SetDesiredThetaDeg(traj_des(i));
					break;
				}
				case JOINT_ARRAY_MODE_SPLINED_TRAJ:
				{
					joints[i]->SetDesiredControlMode(JOINT_MODE_THETA);	
					joints[i]->SetDesiredThetaDeg(traj_des(i));
					break;
				}
				default:
					joints[i]->SetDesiredControlMode(JOINT_MODE_OFF);
			};
		}
	}
}

void M3JointArray::StepStatus()
{	
	if (IsStateError())
		return;
	for (int i=0;i<ndof;i++)
	{
		if (joints[i]!=NULL)
		{
			status.set_motor_temp(i,joints[i]->GetMotorTemp());
			status.set_amp_temp(i,joints[i]->GetAmpTemp());
			status.set_current(i,joints[i]->GetCurrent());
			status.set_torque(i,joints[i]->GetTorque());
			status.set_torquedot(i,joints[i]->GetTorqueDot());
			status.set_torquedot(i,joints[i]->GetTorqueDot());
			status.set_theta(i,joints[i]->GetThetaDeg());
			status.set_thetadot(i,joints[i]->GetThetaDotDeg());
			status.set_thetadotdot(i,joints[i]->GetThetaDotDotDeg());
			status.set_pwm_cmd(i,joints[i]->GetPwmCmd());
			status.set_flags(i,joints[i]->GetFlags());
			torque(i)=status.torque(i);
			theta(i)=status.theta(i);
			thetadot(i)=status.thetadot(i);
			thetadotdot(i)=status.thetadotdot(i);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3JointArray::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	doc["ndof"] >> ndof;
	for(YAML::Iterator it=doc["joint_components"].begin();it!=doc["joint_components"].end();++it) 
	{
   		string key, value;
    		it.first() >> key;
    		it.second() >> value;
		int id=atoi(key.substr(1).c_str()); //"J0" gives 0, etc
		if (joint_names.size() <= (id+1))
		{
			joint_names.resize(id+1);
		}
		joint_names[id]=value;
	}
	
	return true;
}

ServiceServer M3JointArray::RosInitCmd(NodeHandle * node_handle)
{	
	return node_handle->advertiseService(GetName()+"_cmd",&M3JointArray::RosCallbackCmd, this);		
}


ServiceServer M3JointArray::RosInitStatus(NodeHandle * node_handle)
{
	return node_handle->advertiseService(GetName()+"_status",&M3JointArray::RosCallbackStatus, this);
}

ServiceServer M3JointArray::RosInitParam(NodeHandle * node_handle)
{
	return node_handle->advertiseService(GetName()+"_param",&M3JointArray::RosCallbackParam, this);
}

bool M3JointArray::RosCallbackCmd(m3meka_msgs::M3JointArrayCmd::Request  &req, m3meka_msgs::M3JointArrayCmd::Response &res)
{
  for (int i=0;i<ndof;i++)
  {
    command.set_tq_desired(i, req.tq_desired[i]);
    command.set_pwm_desired(i, req.pwm_desired[i]);
    command.set_ctrl_mode(i, (JOINT_ARRAY_MODE)req.ctrl_mode[i]);
    command.set_q_desired(i, req.q_desired[i]);
    command.set_qdot_desired(i, req.qdot_desired[i]);    
    command.set_q_slew_rate(i, req.q_slew_rate[i]);        
  }
  return true;
}

bool M3JointArray::RosCallbackStatus(m3meka_msgs::M3JointArrayStatus::Request  &req, m3meka_msgs::M3JointArrayStatus::Response &res)
{
  res.motor_temp.resize(GetNumDof());
  res.amp_temp.resize(GetNumDof());
  res.current.resize(GetNumDof());
  res.torque.resize(GetNumDof());
  res.torquedot.resize(GetNumDof());
  res.theta.resize(GetNumDof());
  res.thetadot.resize(GetNumDof());
  res.thetadotdot.resize(GetNumDof());
  res.pwm_cmd.resize(GetNumDof());
  
  for (int i=0;i<ndof;i++)
  {
    res.motor_temp[i] = status.motor_temp(i);
    res.amp_temp[i] = status.amp_temp(i);
    res.current[i] = status.current(i);
    res.torque[i] = status.torque(i);
    res.torquedot[i] = status.torquedot(i);
    res.theta[i] = status.theta(i);
    res.thetadot[i] = status.thetadot(i);
    res.thetadotdot[i] = status.thetadotdot(i);
    res.pwm_cmd[i] = status.pwm_cmd(i);
  }

  res.completed_spline_idx = status.completed_spline_idx();
  
  return true;
}

bool M3JointArray::RosCallbackParam(m3meka_msgs::M3JointArrayParam::Request  &req, m3meka_msgs::M3JointArrayParam::Response &res)
{
  return true;
}


}