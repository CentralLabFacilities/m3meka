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

#include "m3/hardware/motor.h"
#include "math.h"
#include <cmath>
namespace m3{
	
using namespace std;


void M3MotorModel::ReadConfig(const YAML::Node & doc)
{
	string t;
	doc["name"] >> name;
	try 
	{
		doc["model_type"] >> t;
		if (t.compare("none")==0){model_type=NONE; return;}
		if (t.compare("model_v0")==0){model_type=MODEL_V0;}
		if (t.compare("model_v1")==0){model_type=MODEL_V1;}
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		//M3_WARN("Missing version key in config file for motor %s. Defaulting to type MODEL_V0\n",name.c_str());
		model_type=MODEL_V0;
	} 
	
	if (model_type==MODEL_V0)
	{
		doc["winding_resistance"] >>winding_resistance;
		doc["thermal_resistance_housing_ambient"] >>thermal_resistance_housing_ambient;
		doc["thermal_resistance_rotor_housing"] >>thermal_resistance_rotor_housing;
		doc["max_winding_temp"] >>max_winding_temp;
		doc["gear_ratio"] >>gear_ratio;
		doc["thermal_time_constant_winding"] >>thermal_time_constant_winding; 
		current_sensor_type=CURRENT_MEASURED; //standard for RBL and previous versions
		temp_sensor_type=TEMP_CASE; //standard for RBL and previous versions
		//The winding temp will take thermal_time_constant_winding seconds to achieve a steady-state calculated value
		winding_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_winding);
	}
	if (model_type==MODEL_V1)
	{
		doc["nominal_voltage"] >> nominal_voltage;
		doc["no_load_speed"] >> no_load_speed;
		doc["no_load_current"] >> no_load_current;
		doc["nominal_speed"] >> nominal_speed;
		doc["nominal_torque"] >> nominal_torque;
		doc["nominal_current"] >> nominal_current;
		doc["stall_torque"] >> stall_torque;
		doc["starting_current"] >> starting_current;
		doc["max_efficiency"] >> max_efficiency;
		doc["winding_resistance"] >> winding_resistance;
		doc["winding_inductance"] >> winding_inductance;
		doc["torque_constant"] >> torque_constant;
		doc["speed_constant"] >> speed_constant;
		doc["speed_torque_gradient"] >> speed_torque_gradient;
		doc["mechanical_time_constant"] >> mechanical_time_constant;
		doc["rotor_inertia"] >> rotor_inertia;
		doc["thermal_resistance_housing_ambient"] >> thermal_resistance_housing_ambient;
		doc["thermal_resistance_rotor_housing"] >> thermal_resistance_rotor_housing;
		doc["thermal_time_constant_winding"] >> thermal_time_constant_winding;
		doc["thermal_time_constant_motor"] >> thermal_time_constant_motor;
		doc["max_winding_temp"] >> max_winding_temp;
		doc["gear_ratio"] >> gear_ratio;
		doc["max_pwm_duty"] >> max_pwm_duty;
		safe_pwm_duty=max_pwm_duty;
		safe_pwm_slew.Reset(max_pwm_duty);
		try 
		{
			doc["safe_thermal_pct"]>>safe_thermal_pct;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_thermal_pct=0.80;
		} 
		try 
		{
			doc["safe_pwm_pct"]>>safe_pwm_pct;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_pwm_pct=0.90;
		} 
		try 
		{
			doc["i_scale"]>>i_scale;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			i_scale=1.0;
		} 
		doc["amplifier_resistance"] >> amplifier_resistance;
		doc["current_sensor_type"]>>t;
		if (t.compare("none")==0){current_sensor_type=CURRENT_NONE;} 
		if (t.compare("measured")==0){current_sensor_type=CURRENT_MEASURED;} 
		if (t.compare("controlled")==0){current_sensor_type=CURRENT_CONTROLLED;} 
		doc["temp_sensor_type"]>>t;
		if (t.compare("none")==0){temp_sensor_type=TEMP_NONE;} 
		if (t.compare("ambient")==0){temp_sensor_type=TEMP_AMBIENT;} 
		if (t.compare("case")==0){temp_sensor_type=TEMP_CASE;} 
		//The winding temp will take thermal_time_constant_winding seconds to achieve a steady-state calculated value
		winding_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_winding);
		case_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_motor);
	}

	
	//1/2 sec for rms current estimates.
	//5 sec for continuous current estimates
	int window_us=0.5*1000000;
	int downsample = (window_us/RT_TASK_FREQUENCY)/100;
	curr_avg_rms.Resize(window_us, downsample);
	window_us=5.0*1000000;
	downsample = (window_us/RT_TASK_FREQUENCY)/100;
	curr_avg_cont.Resize(window_us, downsample);
	//Report back a 1s filtered temp sensor
	window_us=1.0*1000000;
	downsample = (window_us/RT_TASK_FREQUENCY)/100;
	tmp_avg.Resize(window_us,downsample);
	
}

//Using cold resistance
mReal  M3MotorModel::mNmToPwm(mReal mNm)
{
    mReal i_a = (mNm/torque_constant)/gear_ratio;
    mReal duty = (i_a*winding_resistance+v_cemf)/nominal_voltage;
    duty=CLAMP(duty,-1.0,1.0);
    mReal pwm = duty*max_pwm_duty;
    //if (tmp_cnt++%100==0)
    //  M3_INFO("mNm: %f iA: %f v_cemf: %f vreq: %f, duty: %f pwm %f\n", mNm, i_a, v_cemf, i_a*winding_resistance, duty, pwm);
    return pwm;
}

//Input sensor data,if present, of current sensor, amplifier pwm, velocity, and temp sensor
void M3MotorModel::Step(mReal i, mReal pwm, mReal rpm, mReal tmp)
{
	if (model_type==NONE)
	  return;
	/////////////// Voltage terms //////////////////////////////
	mReal duty=0;
	v_pwm = 0;
	v_cemf =0;
	power_mech=0;
	power_elec=0;
	if (model_type==MODEL_V1)
	{
		duty=pwm/max_pwm_duty ;
		v_pwm = ABS(nominal_voltage*pwm/max_pwm_duty);
		v_cemf = ABS(rpm*gear_ratio/speed_constant);
	}
	/////////////// Estimate current ////////////////////////////
	//Current controlled amplifier, Use the commanded value as the 'sensed' value
	if (current_sensor_type==CURRENT_CONTROLLED)
	{
		i_rms=i;
		i_cont=i;
	}
	//Current measurement available from the amplifier
	//i_cont is longer time scale than i_rms
	if (current_sensor_type==CURRENT_MEASURED)
	{
	  if (model_type==MODEL_V0)
		i_rms=sqrt(ABS(curr_avg_rms.Step(i*i))); //Just filter (Avoid NAN)
	  if (model_type==MODEL_V1)
		i_rms=min(starting_current*1000.0,sqrt(ABS(curr_avg_rms.Step(i*i)))); //Just filter (Avoid NAN)
	  i_cont=curr_avg_cont.Step(i_rms);
	}
	
	//No current sensing available. Estimate it based on the applied voltage and back-EMF
	if (current_sensor_type==CURRENT_NONE && model_type==MODEL_V1)
	{
		//Basic motor model with back emf. Estimates current from applied duty cycle and motor velocity.
		//See paper: "Towards a dynamic actuator model for a hexapod robot"	
		//I_a = (d*Vs-Ks*omega)/(Ra+d*d*Ramp)
		//duty should be -1.0 to 1.0
		//put in mA
		
		mReal I_ma = MAX(.00001,((MAX(0.0,v_pwm-v_cemf))/(winding_resistance+duty*duty*amplifier_resistance))*1000.0);
		mReal x=curr_avg_rms.Step(I_ma*I_ma); //for some reason must brake up term and be >0, otherwise get NAN when I_ma=0
		
		i_rms = i_scale*sqrt(ABS(x)); //min(starting_current*1000.0,sqrt(curr_avg_rms.Step(I_ma*I_ma)));
		//if (tmp_cnt++%100==0)
		//	M3_INFO("i_rms %f x %f I_ma %f v_pwm %f v_cemf %f duty %f\n",i_rms,x,I_ma, v_pwm, v_cemf, duty);
		i_cont=curr_avg_cont.Step(i_rms);
	}
	
	if (model_type==MODEL_V1 )
	{
		power_mech=(i_rms/1000.0)*v_cemf;
		power_elec=(i_rms/1000.0)*v_pwm;
	}
	
	/////////////// Estimate winding temp ///////////////////////
	//Get Case/Ambient temp
	ambient_temp=25.0; //provide default in case no sensor present
	case_temp=25.0;
	winding_temp=25.0;
	
	if (first_wt_step)
		tmp_avg.Reset(25.0);
		
	//Newer hardware has ambient temp sensors
	if (temp_sensor_type==TEMP_AMBIENT) 
	{
		if (first_wt_step)
		{
			tmp_avg.Reset(tmp);
		}
		ambient_temp=tmp_avg.Step(tmp);
	}
	
	//Older hardware has case temp sensors
	if (temp_sensor_type==TEMP_CASE)
	{
		if (first_wt_step)
			tmp_avg.Reset(tmp);
		case_temp=tmp_avg.Step(tmp);
		ambient_temp=case_temp; //w/o greater knowledge, assume that internal body temp follows motor case temp
		
	}
	
	//Estimate winding resistance (from maxon catalog)
	mReal alpha_cu=.00393; //copper constant
	mReal hot_resistance = winding_resistance*(1.0+alpha_cu*(winding_temp-25.0));
	//Estimate winding temp
	power_heat=(i_rms/1000.0)*(i_rms/1000.0)*hot_resistance;
	if (temp_sensor_type==TEMP_CASE)
	{
		mReal val=(thermal_resistance_rotor_housing)*power_heat+case_temp;
		if (first_wt_step)
			winding_temp_avg.Reset(val);
		winding_temp = winding_temp_avg.Step(val);
	}
	
	if ((temp_sensor_type==TEMP_AMBIENT || temp_sensor_type==TEMP_NONE)&& model_type==MODEL_V1)
	{
		mReal val_c=ambient_temp+thermal_resistance_rotor_housing*power_heat;
		mReal val_w=(thermal_resistance_rotor_housing+thermal_resistance_housing_ambient)*power_heat+ambient_temp;
		if (first_wt_step)
		{
			winding_temp_avg.Reset(val_w);
			case_temp_avg.Reset(val_c);
		}
		case_temp = case_temp_avg.Step(val_c); //Exponential filter with thermal time constant
		winding_temp = winding_temp_avg.Step(val_w);//Exponential filter with thermal time constant
	}
	
	//When the winding is thermal_limit_pct% of over-temp, limit the voltage to the motor
	//to a value that results in the safe 90% of max continuous current
	//slew to the limit over thermal_time_constant_winding seconds 
	//restore to max over 5 x thermal_time_constant_winding seconds (allow extra time for motor to cool down)
	mReal mpa=safe_pwm_pct*max_pwm_duty*nominal_torque/stall_torque;
	mReal rate_down = (max_pwm_duty-mpa)/MIN(thermal_time_constant_winding,1.0);
	mReal rate_up = (max_pwm_duty-mpa)/(5*thermal_time_constant_winding);
	if (winding_temp>safe_thermal_pct*max_winding_temp)
	{
		safe_pwm_duty=safe_pwm_slew.Step(mpa,rate_down);
	}
	else
		safe_pwm_duty=safe_pwm_slew.Step(max_pwm_duty,rate_up);
	
	first_wt_step=MAX(0,first_wt_step-1);//This resets for first few cycles to ensure start with valid data
}

} //namespace
///////////////////////////////////////////////////////