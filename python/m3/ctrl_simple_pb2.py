# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import component_base_pb2
import actuator_pb2

DESCRIPTOR = descriptor.FileDescriptor(
  name='ctrl_simple.proto',
  package='',
  serialized_pb='\n\x11\x63trl_simple.proto\x1a\x14\x63omponent_base.proto\x1a\x0e\x61\x63tuator.proto\"k\n\x19M3CtrlSimpleStatusCommand\x12\r\n\x05theta\x18\x01 \x01(\x01\x12\x0f\n\x07\x63urrent\x18\x02 \x01(\x01\x12\x0e\n\x06torque\x18\x03 \x01(\x01\x12\x11\n\tstiffness\x18\x04 \x01(\x01\x12\x0b\n\x03pwm\x18\x05 \x01(\x01\"\x9b\x01\n\x12M3CtrlSimpleStatus\x12\x1b\n\x04\x62\x61se\x18\x01 \x01(\x0b\x32\r.M3BaseStatus\x12+\n\x07\x63ommand\x18\x02 \x01(\x0b\x32\x1a.M3CtrlSimpleStatusCommand\x12#\n\x08\x61\x63tuator\x18\x03 \x01(\x0b\x32\x11.M3ActuatorStatus\x12\x16\n\x0etorque_gravity\x18\x04 \x01(\x01\"<\n\x11M3ParamTrajectory\x12\x0c\n\x04\x66req\x18\x01 \x01(\x01\x12\x0b\n\x03\x61mp\x18\x02 \x01(\x01\x12\x0c\n\x04zero\x18\x03 \x01(\x01\"\xcf\x01\n\x11M3CtrlSimpleParam\x12(\n\x0ctraj_current\x18\x01 \x01(\x0b\x32\x12.M3ParamTrajectory\x12\'\n\x0btraj_torque\x18\x02 \x01(\x0b\x32\x12.M3ParamTrajectory\x12&\n\ntraj_theta\x18\x03 \x01(\x0b\x32\x12.M3ParamTrajectory\x12\x1e\n\tpid_theta\x18\x04 \x01(\x0b\x32\x0b.M3ParamPID\x12\x1f\n\npid_torque\x18\x05 \x01(\x0b\x32\x0b.M3ParamPID\"Y\n\nM3ParamPID\x12\x0b\n\x03k_p\x18\x01 \x01(\x01\x12\x0b\n\x03k_i\x18\x02 \x01(\x01\x12\x0b\n\x03k_d\x18\x03 \x01(\x01\x12\x11\n\tk_i_limit\x18\x04 \x01(\x01\x12\x11\n\tk_i_range\x18\x05 \x01(\x01\"\xc9\x01\n\x13M3CtrlSimpleCommand\x12$\n\tctrl_mode\x18\x01 \x01(\x0e\x32\x11.CTRL_SIMPLE_MODE\x12)\n\ttraj_mode\x18\x02 \x01(\x0e\x32\x16.CTRL_SIMPLE_TRAJ_MODE\x12\x17\n\x0f\x64\x65sired_current\x18\x03 \x01(\x01\x12\x15\n\rdesired_theta\x18\x04 \x01(\x01\x12\x16\n\x0e\x64\x65sired_torque\x18\x05 \x01(\x01\x12\x19\n\x11\x64\x65sired_stiffness\x18\x06 \x01(\x01*\xae\x01\n\x10\x43TRL_SIMPLE_MODE\x12\x11\n\rCTRL_MODE_OFF\x10\x00\x12\x15\n\x11\x43TRL_MODE_CURRENT\x10\x01\x12\x14\n\x10\x43TRL_MODE_TORQUE\x10\x02\x12\x17\n\x13\x43TRL_MODE_TORQUE_GC\x10\x03\x12\x13\n\x0f\x43TRL_MODE_THETA\x10\x04\x12\x17\n\x13\x43TRL_MODE_THETA_IMP\x10\x05\x12\x13\n\x0f\x43TRL_MODE_BRAKE\x10\x06*E\n\x15\x43TRL_SIMPLE_TRAJ_MODE\x12\x0c\n\x08TRAJ_OFF\x10\x00\x12\x0f\n\x0bTRAJ_SQUARE\x10\x01\x12\r\n\tTRAJ_SINE\x10\x02\x42\x02H\x01')

_CTRL_SIMPLE_MODE = descriptor.EnumDescriptor(
  name='CTRL_SIMPLE_MODE',
  full_name='CTRL_SIMPLE_MODE',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_OFF', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_CURRENT', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_TORQUE', index=2, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_TORQUE_GC', index=3, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_THETA', index=4, number=4,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_THETA_IMP', index=5, number=5,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='CTRL_MODE_BRAKE', index=6, number=6,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=894,
  serialized_end=1068,
)


_CTRL_SIMPLE_TRAJ_MODE = descriptor.EnumDescriptor(
  name='CTRL_SIMPLE_TRAJ_MODE',
  full_name='CTRL_SIMPLE_TRAJ_MODE',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='TRAJ_OFF', index=0, number=0,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='TRAJ_SQUARE', index=1, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='TRAJ_SINE', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1070,
  serialized_end=1139,
)


CTRL_MODE_OFF = 0
CTRL_MODE_CURRENT = 1
CTRL_MODE_TORQUE = 2
CTRL_MODE_TORQUE_GC = 3
CTRL_MODE_THETA = 4
CTRL_MODE_THETA_IMP = 5
CTRL_MODE_BRAKE = 6
TRAJ_OFF = 0
TRAJ_SQUARE = 1
TRAJ_SINE = 2



_M3CTRLSIMPLESTATUSCOMMAND = descriptor.Descriptor(
  name='M3CtrlSimpleStatusCommand',
  full_name='M3CtrlSimpleStatusCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='theta', full_name='M3CtrlSimpleStatusCommand.theta', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='current', full_name='M3CtrlSimpleStatusCommand.current', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='torque', full_name='M3CtrlSimpleStatusCommand.torque', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='stiffness', full_name='M3CtrlSimpleStatusCommand.stiffness', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='pwm', full_name='M3CtrlSimpleStatusCommand.pwm', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=59,
  serialized_end=166,
)


_M3CTRLSIMPLESTATUS = descriptor.Descriptor(
  name='M3CtrlSimpleStatus',
  full_name='M3CtrlSimpleStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='base', full_name='M3CtrlSimpleStatus.base', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='command', full_name='M3CtrlSimpleStatus.command', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='actuator', full_name='M3CtrlSimpleStatus.actuator', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='torque_gravity', full_name='M3CtrlSimpleStatus.torque_gravity', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=169,
  serialized_end=324,
)


_M3PARAMTRAJECTORY = descriptor.Descriptor(
  name='M3ParamTrajectory',
  full_name='M3ParamTrajectory',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='freq', full_name='M3ParamTrajectory.freq', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='amp', full_name='M3ParamTrajectory.amp', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='zero', full_name='M3ParamTrajectory.zero', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=326,
  serialized_end=386,
)


_M3CTRLSIMPLEPARAM = descriptor.Descriptor(
  name='M3CtrlSimpleParam',
  full_name='M3CtrlSimpleParam',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='traj_current', full_name='M3CtrlSimpleParam.traj_current', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='traj_torque', full_name='M3CtrlSimpleParam.traj_torque', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='traj_theta', full_name='M3CtrlSimpleParam.traj_theta', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='pid_theta', full_name='M3CtrlSimpleParam.pid_theta', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='pid_torque', full_name='M3CtrlSimpleParam.pid_torque', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=389,
  serialized_end=596,
)


_M3PARAMPID = descriptor.Descriptor(
  name='M3ParamPID',
  full_name='M3ParamPID',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='k_p', full_name='M3ParamPID.k_p', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='k_i', full_name='M3ParamPID.k_i', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='k_d', full_name='M3ParamPID.k_d', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='k_i_limit', full_name='M3ParamPID.k_i_limit', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='k_i_range', full_name='M3ParamPID.k_i_range', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=598,
  serialized_end=687,
)


_M3CTRLSIMPLECOMMAND = descriptor.Descriptor(
  name='M3CtrlSimpleCommand',
  full_name='M3CtrlSimpleCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='ctrl_mode', full_name='M3CtrlSimpleCommand.ctrl_mode', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='traj_mode', full_name='M3CtrlSimpleCommand.traj_mode', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='desired_current', full_name='M3CtrlSimpleCommand.desired_current', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='desired_theta', full_name='M3CtrlSimpleCommand.desired_theta', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='desired_torque', full_name='M3CtrlSimpleCommand.desired_torque', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='desired_stiffness', full_name='M3CtrlSimpleCommand.desired_stiffness', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=690,
  serialized_end=891,
)

_M3CTRLSIMPLESTATUS.fields_by_name['base'].message_type = component_base_pb2._M3BASESTATUS
_M3CTRLSIMPLESTATUS.fields_by_name['command'].message_type = _M3CTRLSIMPLESTATUSCOMMAND
_M3CTRLSIMPLESTATUS.fields_by_name['actuator'].message_type = actuator_pb2._M3ACTUATORSTATUS
_M3CTRLSIMPLEPARAM.fields_by_name['traj_current'].message_type = _M3PARAMTRAJECTORY
_M3CTRLSIMPLEPARAM.fields_by_name['traj_torque'].message_type = _M3PARAMTRAJECTORY
_M3CTRLSIMPLEPARAM.fields_by_name['traj_theta'].message_type = _M3PARAMTRAJECTORY
_M3CTRLSIMPLEPARAM.fields_by_name['pid_theta'].message_type = _M3PARAMPID
_M3CTRLSIMPLEPARAM.fields_by_name['pid_torque'].message_type = _M3PARAMPID
_M3CTRLSIMPLECOMMAND.fields_by_name['ctrl_mode'].enum_type = _CTRL_SIMPLE_MODE
_M3CTRLSIMPLECOMMAND.fields_by_name['traj_mode'].enum_type = _CTRL_SIMPLE_TRAJ_MODE
DESCRIPTOR.message_types_by_name['M3CtrlSimpleStatusCommand'] = _M3CTRLSIMPLESTATUSCOMMAND
DESCRIPTOR.message_types_by_name['M3CtrlSimpleStatus'] = _M3CTRLSIMPLESTATUS
DESCRIPTOR.message_types_by_name['M3ParamTrajectory'] = _M3PARAMTRAJECTORY
DESCRIPTOR.message_types_by_name['M3CtrlSimpleParam'] = _M3CTRLSIMPLEPARAM
DESCRIPTOR.message_types_by_name['M3ParamPID'] = _M3PARAMPID
DESCRIPTOR.message_types_by_name['M3CtrlSimpleCommand'] = _M3CTRLSIMPLECOMMAND

class M3CtrlSimpleStatusCommand(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3CTRLSIMPLESTATUSCOMMAND
  
  # @@protoc_insertion_point(class_scope:M3CtrlSimpleStatusCommand)

class M3CtrlSimpleStatus(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3CTRLSIMPLESTATUS
  
  # @@protoc_insertion_point(class_scope:M3CtrlSimpleStatus)

class M3ParamTrajectory(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3PARAMTRAJECTORY
  
  # @@protoc_insertion_point(class_scope:M3ParamTrajectory)

class M3CtrlSimpleParam(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3CTRLSIMPLEPARAM
  
  # @@protoc_insertion_point(class_scope:M3CtrlSimpleParam)

class M3ParamPID(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3PARAMPID
  
  # @@protoc_insertion_point(class_scope:M3ParamPID)

class M3CtrlSimpleCommand(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3CTRLSIMPLECOMMAND
  
  # @@protoc_insertion_point(class_scope:M3CtrlSimpleCommand)

# @@protoc_insertion_point(module_scope)