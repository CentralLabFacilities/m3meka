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

#include "m3/hardware/actuator.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
#include "inttypes.h"

namespace m3{
	
using namespace m3rt;
using namespace std;

///////////////////////////////////////////////////////
#define BOUNDS_AVERAGE_TIME 2.0 //Seconds to average temp values for out-of-bounds errors

void M3Actuator::Startup()
{
	int downsample=20;
	amp_temp_avg.Resize((int)(BOUNDS_AVERAGE_TIME*1000000),downsample);
	
	if (ecc!=NULL)
		SetStateSafeOp();
	else
		SetStateError();
	
	if (!encoder_calib_req)
	  old_is_calibrated = true;
}

void M3Actuator::Shutdown()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3Actuator::ReadConfig(const char * filename)
{	
	int val;
	mReal mval;
	YAML::Node doc;

	if (!M3Component::ReadConfig(filename))
		return false;
	GetYamlDoc(filename, doc);
	
	if (IsVersion(ISS)|| IsVersion(UTA))
	{
		ex_sense.ReadConfig(doc["calib"]["ext_temp"]);
	}
	if (IsVersion(DEFAULT)) //Moved to motor model post version 0
	{	
		doc["param"]["max_motor_temp"] >> max_motor_temp;
		doc["param"]["max_current"] >> max_current;
		ex_sense.ReadConfig(doc["calib"]["motor_temp"]); //motor_temp renamed ext_temp
	}
	if (IsVersion(DEFAULT) || IsVersion(ISS) || IsVersion(UTA))
	{
		doc["ec_component"] >> ecc_name;
		doc["ignore_bounds"] >> val;
		ignore_bounds = (bool) val;
		try
		{
			doc["safe_pwm_limit"] >> val;
			safe_pwm_limit = (bool) val;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_pwm_limit = (bool) 0;
		} 
		doc["param"]["max_amp_temp"] >> mval;
		param.set_max_amp_temp(mval);
		doc["param"]["max_tq"] >> mval;
		param.set_max_tq(mval);
		doc["param"]["min_tq"] >> mval;
		param.set_min_tq(mval);
		motor.ReadConfig(doc["calib"]["motor"]);	
		q_sense.ReadConfig( doc["calib"]["theta"]);
		tq_sense.ReadConfig(doc["calib"]["torque"]);
		at_sense.ReadConfig(doc["calib"]["amp_temp"]);
		i_sense.ReadConfig( doc["calib"]["current"]);
		angle_df.ReadConfig( doc["calib"]["angle_df"]);
		torquedot_df.ReadConfig(doc["calib"]["torquedot_df"]);
		try 
		{
			doc["param"]["max_overload_time"] >> mval;
			param.set_max_overload_time(mval);
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			param.set_max_overload_time(1.0);
		} 
		try 
		{		  
			doc["encoder_calib_req"] >> val;
			encoder_calib_req = bool(val);
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			encoder_calib_req=0;
		} 
	}
	if (IsVersion(ISS)||IsVersion(UTA))
	{
		doc["param"]["max_amp_current"] >> mval;
		param.set_max_amp_current(mval);
		
	}
	if (IsVersion(UTA))
	{
		//Trajectory
		doc["param"]["tq_traj"]["freq"] >> mval;
		ParamTorqueTraj()->set_freq(mval);
		doc["param"]["tq_traj"]["amp"] >> mval;
		ParamTorqueTraj()->set_amp(mval);
		doc["param"]["tq_traj"]["zero"] >> mval;
		ParamTorqueTraj()->set_zero(mval);
		
		//Torque IFF Gains
		doc["param"]["pid_tq_iff"]["k_p"] >> val;
		ParamPidTorqueIFF()->set_k_p(val);
		doc["param"]["pid_tq_iff"]["k_i"] >> val;
		ParamPidTorqueIFF()->set_k_i(val);
		doc["param"]["pid_tq_iff"]["k_d"] >> val;
		ParamPidTorqueIFF()->set_k_d(val);
		doc["param"]["pid_tq_iff"]["k_i_limit"] >> val;
		ParamPidTorqueIFF()->set_k_i_limit(val);
		doc["param"]["pid_tq_iff"]["k_i_range"] >> val;
		ParamPidTorqueIFF()->set_k_i_range(val);
	}
	return true;
}

bool M3Actuator::LinkDependentComponents()
{
	ecc=(M3ActuatorEc*) factory->GetComponent(ecc_name);
	if (ecc==NULL)
	{
		M3_INFO("M3ActuatorEc component %s not found for component %s\n",ecc_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//When a QEI encoder is used, unpack the raw data into a tick count
int M3Actuator::GetTicksQEI()
{
		  if (!GetActuatorEc()) return 0;
		    unsigned int low=(unsigned short)((M3ActuatorEcStatus*)ecc->GetStatus())->qei_on();
		    unsigned int high=((unsigned short)((M3ActuatorEcStatus*)ecc->GetStatus())->qei_rollover()<<16);
		    int ticks = high|low;
		    return ticks;
		
}

void M3Actuator::StepStatus()
{
	if (IsStateError())
		return;
	 
	if (IsVersion(DEFAULT) || IsVersion(ISS) || IsVersion(UTA))
	{
		M3ActuatorEcStatus * ecs = (M3ActuatorEcStatus * )(ecc->GetStatus());
		
		
		//Calibrate Raw Data
		
		//M3_INFO("%lu\n",ecs->timestamp()-old_ts);
		//old_ts = ecs->timestamp();
		//Angle
		/*unsigned int low=(unsigned short)ecs->qei_on();
		unsigned int high=(((unsigned short)ecs->qei_rollover())<<16);
		int ticks = high|low;		
		printf("%ld, %lu, %d %hd, %hd\n",(long)(GetTimestamp()-old_ts_rtai), (unsigned long)(ecs->timestamp()-old_ts), ticks-old_ticks, (short)(ecs->timestamp()), (short)ticks);
		//printf("%ld, %lu, %d\n",(long)(GetTimestamp()), (unsigned long)(ecs->timestamp()), ticks);
		old_ticks = ticks;
		old_ts = ecs->timestamp();
		old_ts_rtai = GetTimestamp();*/
		//printf("%lu\n", (unsigned long)(ecs->timestamp()));
		//printf("%ld, %lu, %d\n",(long)(GetTimestamp()), (unsigned long)(ecs->timestamp()), ticks);
		
		q_sense.Step(ecs->qei_on(),ecs->qei_period(),ecs->qei_rollover());
		// Clear state on filters to prevent blow up on encoder zero
		if (!old_is_calibrated && IsEncoderCalibrated() && encoder_calib_req)
		{
		  angle_df.Reset();		 
		}
		old_is_calibrated = IsEncoderCalibrated();
		angle_df.Step(q_sense.GetThetaDeg(),ecs->timestamp());
		status.set_theta(angle_df.GetTheta());
		mReal td=angle_df.GetThetaDot();
		//if (ABS(td)<2.0)
		//  td=0.0;
		status.set_thetadot(td);
		status.set_thetadotdot(angle_df.GetThetaDotDot());
		
		//Torque
		tq_sense.Step(ecs->adc_torque());
		status.set_torque(tq_sense.GetTorque_mNm());
		status.set_torquedot(torquedot_df.Step(status.torque()));
		
		//Peak To Peak measurement
		tq_p2p.Step(GetTorque(),ParamTorqueTraj()->zero());
		status.set_tq_p2p(tq_p2p.GetP2P());
	

		//Current and Temp
		ex_sense.Step(ecs->adc_ext_temp()); //was motor_temp
		at_sense.Step(ecs->adc_amp_temp());
		i_sense.Step(ecs->adc_current_a(),ecs->adc_current_b());
		if (!ecc->IsMotorPowerOn()) //Force zero-reading when off to ignore small bias errors.
		  i_sense.SetZero();
		motor.Step(i_sense.GetCurrent_mA(), GetPwmCmd(), GetThetaDotDeg()*60.0/360.0, ex_sense.GetTempC());//Compute motor 
				
		status.set_amp_temp(amp_temp_avg.Step(at_sense.GetTempC()));
		status.set_motor_temp(motor.GetWindingTemp());
		status.set_ambient_temp(motor.GetAmbientTemp());
		status.set_case_temp(motor.GetCaseTemp());
		status.set_current(motor.GetCurrentRMS());
		status.set_power(motor.GetPowerElec());
		//if (tmp_cnt%100==0)
		 // M3_INFO("%s: %f %f\n",GetName().c_str(),ex_sense.GetTempC(),motor.GetWindingTemp());
		if (safe_pwm_limit && !ignore_bounds)
		{
		  //if (tmp_cnt++%100==0)
		    //M3_INFO("PwmMax: %f\n",motor.GetMaxPwmDuty());
		    ecc->SetPwmMax(motor.GetSafePwmDuty());
		}
			
		//Update status data
		status.set_flags(ecs->flags());
		status.set_pwm_cmd(ecs->pwm_cmd());
		
	}
	StepOverloadDetect();
}

void M3Actuator::StepOverloadDetect()
{
	int overload_cnt_orig=overload_cnt;
	
	if (!ignore_bounds && GetAmpTemp()>param.max_amp_temp())
		overload_cnt++;
	
	if (IsVersion(DEFAULT))
	{
		if (!ignore_bounds && GetCurrent()>max_current)
			overload_cnt++;
		//This is now redundant, leave in for now. Allows to set a more conservative max winding temp.
		if (!ignore_bounds && (GetMotorTemp()>max_motor_temp))
			overload_cnt++;
		if (!ignore_bounds&& motor.GetWindingTemp()>motor.GetMaxWindingTemp())
			overload_cnt++;
	}
	
	if (IsVersion(ISS) || IsVersion(UTA))
	{
		if (!ignore_bounds&& motor.GetWindingTemp()>(motor.GetMaxWindingTemp()))
			overload_cnt++;
		if (!ignore_bounds && motor.GetCurrentContinuous()>param.max_amp_current())
			overload_cnt++;
	}
	if (overload_cnt==overload_cnt_orig) //no overload, reset cntr
	{
		overload_cnt==0;
		return;
	}
	
	if (overload_cnt >= (int)((mReal)param.max_overload_time()*(mReal)RT_TASK_FREQUENCY))
	{
		SetStateError();
		M3_ERR("------ OVER-THRESHOLD EVENT FOR %s ---------------\n",GetName().c_str());
		M3_ERR("Amp temp %f (C)| Threshold of %f (C)\n",GetAmpTemp(),param.max_amp_temp());
		if (IsVersion(DEFAULT))
		{
			M3_ERR("Motor current %f | Threshold of %f \n",GetCurrent(),max_current);
			M3_ERR("Motor temp %f (C) | Threshold of %f (C) \n",GetMotorTemp(),max_motor_temp);
			M3_ERR("Motor winding temp %f (C) | Threshold of %f \n",motor.GetWindingTemp(),motor.GetMaxWindingTemp());
		}
		if (IsVersion(ISS) || IsVersion(UTA))
		{
			M3_ERR("Motor winding temp %f (C) | Threshold of %f (C)\n",motor.GetWindingTemp(),motor.GetMaxWindingTemp());
			M3_ERR("Amplifier current %f |Threshold of %f (mA)\n",motor.GetCurrentContinuous(),param.max_amp_current());
		}
		M3_ERR("------------------------------------------------------\n");
	}
}


void M3Actuator::StepCommand()
{
	status.set_torque_error(0);
	if (!ecc || IsStateSafeOp())
		return;

	M3ActuatorEcCommand * ec_command = (M3ActuatorEcCommand *)ecc->GetCommand();
	M3ActuatorEcParam * ec_param = (M3ActuatorEcParam *)ecc->GetParam();
	ec_command->set_brake_off(command.brake_off());
	
	if(IsStateError())
	{
		ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
		return;
	}
	
	//Inner control loop setpoints
	mReal tq_desired=command.tq_desired();

	if (IsVersion(UTA))
	{
	    //Trajectories
	    mReal tj_dt=GetTimestamp()/1000000.0;//seconds
	    mReal tj_tq=sin(2*M_PI*tj_dt*ParamTorqueTraj()->freq());
	    
	    switch(command.traj_mode())
	    {
	      case TRAJ_TORQUE_SQUARE:
	      {

		if (tj_tq>0)
		  tq_desired=ParamTorqueTraj()->zero()+ParamTorqueTraj()->amp();
		else
		  tq_desired=ParamTorqueTraj()->zero()-ParamTorqueTraj()->amp();
		break;
	      }
	      case TRAJ_TORQUE_SINE:
	      {
		tq_desired=ParamTorqueTraj()->zero()+ParamTorqueTraj()->amp()*tj_tq;
		break;
	      }
	      case TRAJ_OFF:
	      default:
		break;
	    };
	}
	
	//Reset
	status.set_torque_error(0);
	status.set_i_cmd(0);
	status.set_tq_cmd(0);

	switch(command.ctrl_mode())
	{
		case ACTUATOR_MODE_OFF:
			ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
			break;
		case ACTUATOR_MODE_PWM:
			ec_command->set_mode(ACTUATOR_EC_MODE_PWM);
			ec_command->set_t_desire(pwm_dither.Step(command.pwm_desired()));
			break;
		case ACTUATOR_MODE_CURRENT:
			ec_command->set_mode(ACTUATOR_EC_MODE_CURRENT);
			ec_command->set_t_desire(command.i_desired());
			status.set_i_cmd(command.i_desired());
			break;
		case ACTUATOR_MODE_TORQUE:		
		{
			mReal tq_mNm=tq_desired;
			if (tq_sense.IsFFCurrentCtrl())
			{
			  ec_command->set_mode(ACTUATOR_EC_MODE_PWM);
			  tq_mNm=CLAMP(tq_mNm,param.min_tq(),param.max_tq()); //Clamped on DSP as well...
			  ec_command->set_t_desire(tq_sense.mNmToTicks(tq_mNm,&i_sense));
			} else {
			  ec_command->set_mode(ACTUATOR_EC_MODE_TORQUE);
			  int raw_t_max= tq_sense.mNmToTicks(param.max_tq());
			  int raw_t_min= tq_sense.mNmToTicks(param.min_tq());
			  if (raw_t_min>raw_t_max) //Can be reversed depending on calibration
			  {
				  int tmp=raw_t_min;
				  raw_t_min=raw_t_max;
				  raw_t_max=tmp;
			  }
			  //Overwrite the raw  params with the calibrated values
			  ec_param->set_t_max(raw_t_max);
			  ec_param->set_t_min(raw_t_min);			  
			  tq_mNm=CLAMP(tq_mNm,param.min_tq(),param.max_tq()); //Clamped on DSP as well...
			  status.set_torque_error(tq_des_last-GetTorque());
			  tq_des_last=tq_mNm;
			  ec_command->set_t_desire(tq_sense.mNmToTicks(tq_mNm));
			  if (ecc->UseTorqueFF())
			      ecc->SetTorqueFF(motor.mNmToPwm(tq_mNm));
			}
			break;
		}
		case ACTUATOR_MODE_TORQUE_IFF:		
		{
		  if (IsVersion(UTA))
		  {
			mReal tq_mNm=CLAMP(tq_desired,param.min_tq(),param.max_tq()); 
			mReal i_des_ff=ActuatorTorqueToMotorCurrent(tq_mNm);
			mReal i_des_pid =pid_tq_iff.Step(GetTorque(),
						GetTorqueDot(),
						tq_mNm,
						ParamPidTorqueIFF()->k_p(), 
						ParamPidTorqueIFF()->k_i(),
						ParamPidTorqueIFF()->k_d(), 
						ParamPidTorqueIFF()->k_i_limit(),
						ParamPidTorqueIFF()->k_i_range());
			status.set_torque_error(tq_des_last-GetTorque());
			tq_des_last=tq_mNm;
			ec_command->set_t_desire((int)(i_des_pid+i_des_ff));
			ec_command->set_mode(ACTUATOR_EC_MODE_CURRENT); 
			status.set_i_cmd(i_des_pid+i_des_ff);
			status.set_tq_cmd(tq_mNm);
		  }
		  else
		  {
		    ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
		  }
		
			break;
		}
		default:
			ec_command->set_mode(ACTUATOR_EC_MODE_OFF);
			break;
	};
}

}