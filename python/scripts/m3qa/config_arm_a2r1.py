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


# ########################## ACTUATOR_EC A2.R1.J0-J6 ########################################

config_arm_a2r1_actuator_ec={
    'chid': 0,
    'ethercat': {'pdo_version': 'actx1_pdo_v1',
                 'product_code': 1010,
                 'serial_number': 0},
    'name': 'm3actuator_ec_xxx_j0',
    'param': {'config': 2,
              'k_d': 0,
              'k_d_shift': 0,
              'k_ff': 0,
              'k_ff_shift': 0,
              'k_ff_zero': 0,
              'k_i': 0,
              'k_i_limit': 0,
              'k_i_shift': 0,
              'k_p': 0,
              'k_p_shift': 0,
              'pwm_db': 0,
              'pwm_max': 3186,
              'qei_max': 0,
              'qei_min': 0,
              't_max': 0,
              't_min': 0},
    'pwr_component': 'm3pwr_pwr000'}

config_arm_a2r1_actuator_ec_j0=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j1=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j2=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j3=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j4=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j5=config_arm_a2r1_actuator_ec
config_arm_a2r1_actuator_ec_j6=config_arm_a2r1_actuator_ec

# ########################## ACTUATOR A2.R1.J0 ########################################

config_arm_a2r1_actuator_j0={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J1 ########################################

config_arm_a2r1_actuator_j1={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J2 ########################################

config_arm_a2r1_actuator_j2={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J3 ########################################

config_arm_a2r1_actuator_j3={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J4 ########################################

config_arm_a2r1_actuator_j4={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J5 ########################################

config_arm_a2r1_actuator_j5={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## ACTUATOR A2.R1.J6 ########################################

config_arm_a2r1_actuator_j6={
    'calib': {'amp_temp': {'cb_bias': 0.0,
                           'cb_mV_at_25C': 750.0,
                           'cb_mV_per_C': 10.0,
                           'cb_scale': 1.0,
                           'name': 'Microchip TC1047',
                           'type': 'adc_linear_5V'},
              'angle_df': {'theta_df': {'cutoff_freq': 80,
                                        'order': 3,
                                        'type': 'butterworth'},
                           'thetadot_df': {'cutoff_freq': 20,
                                           'order': 3,
                                           'type': 'diff_butterworth'},
                           'thetadotdot_df': {'cutoff_freq': 20,
                                              'order': 3,
                                              'type': 'diff_butterworth'},
                           'type': 'df_chain'},
              'current': {'cb_bias': 0.0,
                          'cb_mV_per_A': 100.0,
                          'cb_scale': 1.0,
                          'cb_ticks_at_zero_a': 2048,
                          'cb_ticks_at_zero_b': 2048,
                          'name': 'Allegro ACS712-20',
                          'type': 'adc_linear_5V'},
              'motor': {'gear_ratio': 120.0,
                        'max_winding_temp': 155,
                        'name': 'Maxon RE40 150W 24V',
                        'thermal_resistance_housing_ambient': 4.7000000000000002,
                        'thermal_resistance_rotor_housing': 1.8999999999999999,
                        'thermal_time_constant_winding': 41.0,
                        'winding_resistance': 0.316},
              'motor_temp': {'cb_bias': 0.0,
                             'cb_mV_at_25C': 750.0,
                             'cb_mV_per_C': 10.0,
                             'cb_scale': 1.0,
                             'name': 'Analog TMP36',
                             'type': 'adc_linear_3V3'},
              'theta': {'cb_bias': 0.0,
                        'cb_scale': 1.0,
                        'name': 'ContElec VertX13',
                        'type': 'vertx_14bit'},
              'torque': {'cb_bias': 0.0,
                         'cb_inv_torque': [1.0,0.0],
                         'cb_scale': 1.0,
                         'cb_torque': [1.0,0.0],
                         'name': 'ContElec VertX13',
                         'type': 'sea_vertx_14bit'},
              'torquedot_df': {'cutoff_freq': 100,
                               'order': 3,
                               'type': 'diff_butterworth'}},
    'description': 'max2_v0.1_seax2_v1.1',
    'ec_component': 'm3actuator_ec_xxx_jx',
    'ignore_bounds': 0,
    'joint_component': 'm3joint_xxx_jx',
    'name': 'm3actuator_xxx_jx',
    'param': {'max_amp_temp': 100.0,
              'max_current': 12000,
              'max_motor_temp': 145.0,
              'max_tq': 40000.0,
              'min_tq': -40000.0,
              'thetadot_deadband': 2.0}}

# ########################## JOINT A2.R1.J0-J4 ########################################

config_arm_a2r1_joint_j0={
    'actuator_component': 'm3actuator_xxx_j0', 
    'name': 'm3joint_xxx_j0',
    'param': {'kq_d': 0.0,
              'kq_g': 1.0,
              'kq_i': 0.0,
              'kq_i_limit': 0.0,
              'kq_i_range': 0.0,
              'kq_p': 0.0,
              'kt_d': 0.0,
              'kt_i': 0.0,
              'kt_i_limit': 0.0,
              'kt_i_range': 0.0,
              'kt_p': 0.0,
              'max_q': 360.0,
              'max_q_pad': 0.0,
              'max_q_slew_rate': 25.0,
              'min_q': 0.0,
              'min_q_pad': 0.0},
    'transmission':{'act_name': 'm3actuator_xxx_j0',
                    'qj_to_qa':[1.0],
                    'qs_to_qj':[1.0],
                    'tqj_to_tqa':[1.0],
                    'tqs_to_tqj':[1.0],
                    'type': 'gear'}}

config_arm_a2r1_joint_j1=config_arm_a2r1_joint_j0
config_arm_a2r1_joint_j2=config_arm_a2r1_joint_j0
config_arm_a2r1_joint_j3=config_arm_a2r1_joint_j0
config_arm_a2r1_joint_j4=config_arm_a2r1_joint_j0

# ########################## JOINT A2.R1.J5 ########################################

config_arm_a2r1_joint_j5={
    'actuator_component': 'm3actuator_xxx_j5', 
    'name': 'm3joint_xxx_j0',
    'param': {'kq_d': 0.0,
              'kq_g': 1.0,
              'kq_i': 0.0,
              'kq_i_limit': 0.0,
              'kq_i_range': 0.0,
              'kq_p': 0.0,
              'kt_d': 0.0,
              'kt_i': 0.0,
              'kt_i_limit': 0.0,
              'kt_i_range': 0.0,
              'kt_p': 0.0,
              'max_q': 360.0,
              'max_q_pad': 0.0,
              'max_q_slew_rate': 25.0,
              'min_q': 0.0,
              'min_q_pad': 0.0},
    'transmission':{'act_name': 'm3actuator_xxx_j5',
                    'cpj_name': 'm3joint_xxx_j6',
                    'qj_to_qa':[2.0,2.0],
                    'qs_to_qj':[1.0,0.0],
                    'tqj_to_tqa':[0.25,0.25],
                    'tqs_to_tqj':[2.0,2.0],
                    'type': 'differential'}}

# ########################## JOINT A2.R1.J6 ########################################

config_arm_a2r1_joint_j6={
    'actuator_component': 'm3actuator_xxx_j6', 
    'name': 'm3joint_xxx_j0',
    'param': {'kq_d': 0.0,
              'kq_g': 1.0,
              'kq_i': 0.0,
              'kq_i_limit': 0.0,
              'kq_i_range': 0.0,
              'kq_p': 0.0,
              'kt_d': 0.0,
              'kt_i': 0.0,
              'kt_i_limit': 0.0,
              'kt_i_range': 0.0,
              'kt_p': 0.0,
              'max_q': 360.0,
              'max_q_pad': 0.0,
              'max_q_slew_rate': 25.0,
              'min_q': 0.0,
              'min_q_pad': 0.0},
    'transmission':{'act_name': 'm3actuator_xxx_j6',
                    'cpj_name': 'm3joint_xxx_j5',
                    'qj_to_qa':[-2.0,2.0],
                    'qs_to_qj':[1.0,0.0],
                    'tqj_to_tqa':[-0.25,0.25],
                    'tqs_to_tqj':[-2.0,2.0],
                    'type': 'differential'}}

# ########################## ARM A2.R1 ########################################

config_arm_a2r1={
    'name': 'm3arm_xxx',
    'ndof': 7,
    'limb_name': 'xxx_arm',
    'dynamatics_component': 'm3dynamatics_xxx',
'joint_components':{'J0': 'm3joint_xxx_j0',
                    'J1': 'm3joint_xxx_j1',
                    'J2': 'm3joint_xxx_j2',
                    'J3': 'm3joint_xxx_j3',
                    'J4': 'm3joint_xxx_j4',
                    'J5': 'm3joint_xxx_j5',
                    'J6': 'm3joint_xx_j6'}}

# ########################## Dynamatics Right Arm A2.R1 ########################################

config_arm_a2r1_dynamatics_right_arm={
    'chain_component': 'm3arm_xxx',
    'links': [{'Ixx': 0.00465797,
               'Ixy': 4.7899999999999999e-06,
               'Ixz': 9.9699999999999994e-06,
               'Iyy': 0.00374632,
               'Iyz': 2.3819999999999999e-05,
               'Izz': 0.0028815500000000001,
               'a': 0.0,
               'alpha': 90.0,
               'cx': 0.0057061799999999999,
               'cy': -0.0028646800000000001,
               'cz': -0.021885080000000001,
               'd': 0.18465000000000001,
               'joint_offset': -90,
               'm': 1.9910000000000001},
              {'Ixx': 0.00199343,
               'Ixy': -0.00076418000000000005,
               'Ixz': -6.9500000000000004e-06,
               'Iyy': 0.00122781,
               'Iyz': 1.8309999999999999e-05,
               'Izz': 0.0021946800000000001,
               'a': 0.0,
               'alpha': 90,
               'cx': 0.0256435,
               'cy': -0.04287067,
               'cz': -0.00098167000000000003,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.504},
              {'Ixx': 0.02883807,
               'Ixy': 2.921e-05,
               'Ixz': -0.0016239,
               'Iyy': 0.029054839999999998,
               'Iyz': -6.9670000000000002e-05,
               'Izz': 0.0022021599999999999,
               'a': 0.03175,
               'alpha': 90.0,
               'cx': 0.0052489099999999999,
               'cy': 0.00101117,
               'cz': -0.08821909,
               'd': 0.27865000000000001,
               'joint_offset': 90.0,
               'm': 2.2724299999999999},
              {'Ixx': 0.00056886000000000003,
               'Ixy': 2.7000000000000001e-07,
               'Ixz': 3.1e-07,
               'Iyy': 0.00040234,
               'Iyz': 1.065e-05,
               'Izz': 0.00035205000000000002,
               'a': -0.0063499999999999997,
               'alpha': 90,
               'cx': -4.0139999999999999e-05,
               'cy': 0.02541156,
               'cz': -0.002758,
               'd': 0,
               'joint_offset': 0.0,
               'm': 0.19700000000000001},
              {'Ixx': 0.018770990000000001,
               'Ixy': -4.9999999999999998e-07,
               'Ixz': -0.0010734200000000001,
               'Iyy': 0.018805539999999999,
               'Iyz': 7.5149999999999997e-05,
               'Izz': 0.0010869600000000001,
               'a': 0.0,
               'alpha': -90,
               'cx': 0.0078302100000000006,
               'cy': -0.00024240000000000001,
               'cz': -0.1170596,
               'd': 0.27045999999999998,
               'joint_offset': 0,
               'm': 1.3400000000000001},
              {'Ixx': 0.00010964,
               'Ixy': -2e-08,
               'Ixz': 0.0,
               'Iyy': 0.00012383000000000001,
               'Iyz': 4.0000000000000001e-08,
               'Izz': 7.7459999999999994e-05,
               'a': 0.0,
               'alpha': 90,
               'cx': 9.8720000000000003e-05,
               'cy': -0.00016891000000000001,
               'cz': -0.0012537,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.22500000000000001},
              {'Ixx': 0.00010018000000000001,
               'Ixy': 8.9999999999999999e-08,
               'Ixz': 1.1000000000000001e-07,
               'Iyy': 6.0600000000000003e-05,
               'Iyz': -5.0200000000000002e-06,
               'Izz': 5.838e-05,
               'a': 0.0,
               'alpha': 90.0,
               'cx': -5.8730000000000002e-05,
               'cy': -0.011888360000000001,
               'cz': 0.0096118499999999999,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.11},
              {'a': 0.0,
               'alpha': 90.0,
               'd': 0.044139999999999999,
               'joint_offset': 0.0}],
    'name': 'm3dynamatics_xxx',
    'ndof': 7,
    'param': {'payload_com': [0.0, 0.0, 0.06],
              'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              'payload_mass': 0.856,
              'use_accelerations': False,
              'use_velocities': False}}
   
# ########################## Dynamatics Left Arm A2.R1 ########################################

config_arm_a2r1_dynamatics_left_arm={
    'chain_component': 'm3arm_xxx',
    'links': [{'Ixx': 0.00569726,
               'Ixy': -2.7000000000000001e-07,
               'Ixz': 1.2999999999999999e-05,
               'Iyy': 0.0036925999999999999,
               'Iyz': -8.2020000000000004e-05,
               'Izz': 0.00292092,
               'a': 0.0,
               'alpha': 90.0,
               'cx': 0.00019141000000000001,
               'cy': -0.015244890000000001,
               'cz': 0.021107089999999998,
               'd': -0.18465000000000001,
               'joint_offset': -90,
               'm': 1.9910000000000001},
              {'Ixx': 0.00199343,
               'Ixy': 0.00076418000000000005,
               'Ixz': -6.9500000000000004e-06,
               'Iyy': 0.00122781,
               'Iyz': -1.8309999999999999e-05,
               'Izz': 0.0021946800000000001,
               'a': 0.0,
               'alpha': 90,
               'cx': -0.0256435,
               'cy': -0.04287067,
               'cz': 2.268e-05,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.504},
              {'Ixx': 0.029535700000000002,
               'Ixy': -2.7549999999999999e-05,
               'Ixz': -0.0017019400000000001,
               'Iyy': 0.029802559999999999,
               'Iyz': 6.8159999999999998e-05,
               'Izz': 0.0021860899999999999,
               'a': -0.03175,
               'alpha': 90.0,
               'cx': 0.0055088300000000002,
               'cy': -0.0014015,
               'cz': -0.089679449999999994,
               'd': 0.27865000000000001,
               'joint_offset': 90.0,
               'm': 2.2724299999999999},
              {'Ixx': 0.00056886000000000003,
               'Ixy': -2.7000000000000001e-07,
               'Ixz': 3.1e-07,
               'Iyy': 0.00040234,
               'Iyz': -1.065e-05,
               'Izz': 0.00035205000000000002,
               'a': -0.0063499999999999997,
               'alpha': 90,
               'cx': 4.0139999999999999e-05,
               'cy': 0.02541156,
               'cz': 0.002758,
               'd': 0,
               'joint_offset': 0.0,
               'm': 0.19700000000000001},
              {'Ixx': 0.018770990000000001,
               'Ixy': -4.9999999999999998e-07,
               'Ixz': -0.0010734200000000001,
               'Iyy': 0.018805539999999999,
               'Iyz': 7.5149999999999997e-05,
               'Izz': 0.0010869600000000001,
               'a': 0.0,
               'alpha': -90,
               'cx': 0.0078302100000000006,
               'cy': -0.00024240000000000001,
               'cz': -0.1170596,
               'd': 0.27045999999999998,
               'joint_offset': 0,
               'm': 1.3400000000000001},
              {'Ixx': 0.00010964,
               'Ixy': -2e-08,
               'Ixz': 0.0,
               'Iyy': 0.00012383000000000001,
               'Iyz': 4.0000000000000001e-08,
               'Izz': 7.7459999999999994e-05,
               'a': 0.0,
               'alpha': 90,
               'cx': 9.8720000000000003e-05,
               'cy': -0.00016891000000000001,
               'cz': -0.0012537,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.22500000000000001},
              {'Ixx': 0.00010018000000000001,
               'Ixy': 8.9999999999999999e-08,
               'Ixz': 1.1000000000000001e-07,
               'Iyy': 6.0600000000000003e-05,
               'Iyz': -5.0200000000000002e-06,
               'Izz': 5.838e-05,
               'a': 0.0,
               'alpha': 90.0,
               'cx': -5.8730000000000002e-05,
               'cy': -0.011888360000000001,
               'cz': 0.0096118499999999999,
               'd': 0.0,
               'joint_offset': 90.0,
               'm': 0.11},
              {'a': 0.0,
               'alpha': 90.0,
               'd': 0.044139999999999999,
               'joint_offset': 0.0}],
    'name': 'm3dynamatics_xxx',
    'ndof': 7,
    'param': {'payload_com': [0.0, 0.0, 0.068],
              'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              'payload_mass': 0.86,
              'use_accelerations': False,
              'use_velocities': False}}
   
# ########################## Payloads A2.R1 ########################################

config_arm_a2r1_payload_h2r1_right_hand_load_cell={
    'payload_com': [0.0, 0.0, 0.068],
    'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'payload_mass': 0.856
    }

config_arm_a2r1_payload_load_cell={
    'payload_com': [0.0, 0.0, 0.008],
    'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'payload_mass': 0.055
    }

config_arm_a2r1_payload_none={
    'payload_com': [0.0, 0.0, 0.000],
    'payload_inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'payload_mass': 0.0
    }

# ########################## A2.R1 ########################################

config_full_a2r1={'actuator_ec':[config_arm_a2r1_actuator_ec_j0,
                                 config_arm_a2r1_actuator_ec_j1,
                                 config_arm_a2r1_actuator_ec_j2,
                                 config_arm_a2r1_actuator_ec_j3,
                                 config_arm_a2r1_actuator_ec_j4,
                                 config_arm_a2r1_actuator_ec_j5,
                                 config_arm_a2r1_actuator_ec_j6],
                  'actuator':[config_arm_a2r1_actuator_j0,
                              config_arm_a2r1_actuator_j1,
                              config_arm_a2r1_actuator_j2,
                              config_arm_a2r1_actuator_j3,
                              config_arm_a2r1_actuator_j4,
                              config_arm_a2r1_actuator_j5,
                              config_arm_a2r1_actuator_j6],
                  'joint':[config_arm_a2r1_joint_j0,
                           config_arm_a2r1_joint_j1,
                           config_arm_a2r1_joint_j2,
                           config_arm_a2r1_joint_j3,
                           config_arm_a2r1_joint_j4,
                           config_arm_a2r1_joint_j5,
                           config_arm_a2r1_joint_j6],
                  'dynamatics_right_arm':config_arm_a2r1_dynamatics_right_arm,
                  'dynamatics_left_arm':config_arm_a2r1_dynamatics_left_arm,
                  'arm':config_arm_a2r1,
                  'payloads':{
                      'h2r1_right_hand_load_cell':config_arm_a2r1_payload_h2r1_right_hand_load_cell,
                      'load_cell':config_arm_a2r1_payload_load_cell,
                      'none':config_arm_a2r1_payload_none}}

