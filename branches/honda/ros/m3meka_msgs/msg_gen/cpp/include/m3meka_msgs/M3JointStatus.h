/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka-new/ros/m3meka_msgs/msg/M3JointStatus.msg */
#ifndef M3MEKA_MSGS_MESSAGE_M3JOINTSTATUS_H
#define M3MEKA_MSGS_MESSAGE_M3JOINTSTATUS_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "m3_msgs/M3BaseStatus.h"

namespace m3meka_msgs
{
template <class ContainerAllocator>
struct M3JointStatus_ : public ros::Message
{
  typedef M3JointStatus_<ContainerAllocator> Type;

  M3JointStatus_()
  : base()
  , motor_temp(0.0)
  , amp_temp(0.0)
  , current(0.0)
  , torque(0.0)
  , torquedot(0.0)
  , theta(0.0)
  , thetadot(0.0)
  , thetadotdot(0.0)
  , torque_gravity(0.0)
  , pwm_cmd(0)
  , ambient_temp(0.0)
  , case_temp(0.0)
  , power(0.0)
  , flags(0)
  {
  }

  M3JointStatus_(const ContainerAllocator& _alloc)
  : base(_alloc)
  , motor_temp(0.0)
  , amp_temp(0.0)
  , current(0.0)
  , torque(0.0)
  , torquedot(0.0)
  , theta(0.0)
  , thetadot(0.0)
  , thetadotdot(0.0)
  , torque_gravity(0.0)
  , pwm_cmd(0)
  , ambient_temp(0.0)
  , case_temp(0.0)
  , power(0.0)
  , flags(0)
  {
  }

  typedef  ::m3_msgs::M3BaseStatus_<ContainerAllocator>  _base_type;
   ::m3_msgs::M3BaseStatus_<ContainerAllocator>  base;

  typedef float _motor_temp_type;
  float motor_temp;

  typedef float _amp_temp_type;
  float amp_temp;

  typedef float _current_type;
  float current;

  typedef float _torque_type;
  float torque;

  typedef float _torquedot_type;
  float torquedot;

  typedef float _theta_type;
  float theta;

  typedef float _thetadot_type;
  float thetadot;

  typedef float _thetadotdot_type;
  float thetadotdot;

  typedef float _torque_gravity_type;
  float torque_gravity;

  typedef int32_t _pwm_cmd_type;
  int32_t pwm_cmd;

  typedef float _ambient_temp_type;
  float ambient_temp;

  typedef float _case_temp_type;
  float case_temp;

  typedef float _power_type;
  float power;

  typedef int32_t _flags_type;
  int32_t flags;


private:
  static const char* __s_getDataType_() { return "m3meka_msgs/M3JointStatus"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "9c6d93ab28413f8b473c8def3d02284b"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "m3_msgs/M3BaseStatus base\n\
float32 motor_temp\n\
float32 amp_temp\n\
float32 current\n\
float32 torque\n\
float32 torquedot\n\
float32 theta\n\
float32 thetadot\n\
float32 thetadotdot\n\
float32 torque_gravity\n\
int32 pwm_cmd\n\
float32 ambient_temp\n\
float32 case_temp\n\
float32 power\n\
int32 flags\n\
\n\
================================================================================\n\
MSG: m3_msgs/M3BaseStatus\n\
string name\n\
uint8 state\n\
int64 timestamp\n\
string rate\n\
string version\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, base);
    ros::serialization::serialize(stream, motor_temp);
    ros::serialization::serialize(stream, amp_temp);
    ros::serialization::serialize(stream, current);
    ros::serialization::serialize(stream, torque);
    ros::serialization::serialize(stream, torquedot);
    ros::serialization::serialize(stream, theta);
    ros::serialization::serialize(stream, thetadot);
    ros::serialization::serialize(stream, thetadotdot);
    ros::serialization::serialize(stream, torque_gravity);
    ros::serialization::serialize(stream, pwm_cmd);
    ros::serialization::serialize(stream, ambient_temp);
    ros::serialization::serialize(stream, case_temp);
    ros::serialization::serialize(stream, power);
    ros::serialization::serialize(stream, flags);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, base);
    ros::serialization::deserialize(stream, motor_temp);
    ros::serialization::deserialize(stream, amp_temp);
    ros::serialization::deserialize(stream, current);
    ros::serialization::deserialize(stream, torque);
    ros::serialization::deserialize(stream, torquedot);
    ros::serialization::deserialize(stream, theta);
    ros::serialization::deserialize(stream, thetadot);
    ros::serialization::deserialize(stream, thetadotdot);
    ros::serialization::deserialize(stream, torque_gravity);
    ros::serialization::deserialize(stream, pwm_cmd);
    ros::serialization::deserialize(stream, ambient_temp);
    ros::serialization::deserialize(stream, case_temp);
    ros::serialization::deserialize(stream, power);
    ros::serialization::deserialize(stream, flags);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(base);
    size += ros::serialization::serializationLength(motor_temp);
    size += ros::serialization::serializationLength(amp_temp);
    size += ros::serialization::serializationLength(current);
    size += ros::serialization::serializationLength(torque);
    size += ros::serialization::serializationLength(torquedot);
    size += ros::serialization::serializationLength(theta);
    size += ros::serialization::serializationLength(thetadot);
    size += ros::serialization::serializationLength(thetadotdot);
    size += ros::serialization::serializationLength(torque_gravity);
    size += ros::serialization::serializationLength(pwm_cmd);
    size += ros::serialization::serializationLength(ambient_temp);
    size += ros::serialization::serializationLength(case_temp);
    size += ros::serialization::serializationLength(power);
    size += ros::serialization::serializationLength(flags);
    return size;
  }

  typedef boost::shared_ptr< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3meka_msgs::M3JointStatus_<ContainerAllocator>  const> ConstPtr;
}; // struct M3JointStatus
typedef  ::m3meka_msgs::M3JointStatus_<std::allocator<void> > M3JointStatus;

typedef boost::shared_ptr< ::m3meka_msgs::M3JointStatus> M3JointStatusPtr;
typedef boost::shared_ptr< ::m3meka_msgs::M3JointStatus const> M3JointStatusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::m3meka_msgs::M3JointStatus_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace m3meka_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9c6d93ab28413f8b473c8def3d02284b";
  }

  static const char* value(const  ::m3meka_msgs::M3JointStatus_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9c6d93ab28413f8bULL;
  static const uint64_t static_value2 = 0x473c8def3d02284bULL;
};

template<class ContainerAllocator>
struct DataType< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3JointStatus";
  }

  static const char* value(const  ::m3meka_msgs::M3JointStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3_msgs/M3BaseStatus base\n\
float32 motor_temp\n\
float32 amp_temp\n\
float32 current\n\
float32 torque\n\
float32 torquedot\n\
float32 theta\n\
float32 thetadot\n\
float32 thetadotdot\n\
float32 torque_gravity\n\
int32 pwm_cmd\n\
float32 ambient_temp\n\
float32 case_temp\n\
float32 power\n\
int32 flags\n\
\n\
================================================================================\n\
MSG: m3_msgs/M3BaseStatus\n\
string name\n\
uint8 state\n\
int64 timestamp\n\
string rate\n\
string version\n\
\n\
\n\
";
  }

  static const char* value(const  ::m3meka_msgs::M3JointStatus_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.base);
    stream.next(m.motor_temp);
    stream.next(m.amp_temp);
    stream.next(m.current);
    stream.next(m.torque);
    stream.next(m.torquedot);
    stream.next(m.theta);
    stream.next(m.thetadot);
    stream.next(m.thetadotdot);
    stream.next(m.torque_gravity);
    stream.next(m.pwm_cmd);
    stream.next(m.ambient_temp);
    stream.next(m.case_temp);
    stream.next(m.power);
    stream.next(m.flags);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3JointStatus_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::m3meka_msgs::M3JointStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::m3meka_msgs::M3JointStatus_<ContainerAllocator> & v) 
  {
    s << indent << "base: ";
s << std::endl;
    Printer< ::m3_msgs::M3BaseStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.base);
    s << indent << "motor_temp: ";
    Printer<float>::stream(s, indent + "  ", v.motor_temp);
    s << indent << "amp_temp: ";
    Printer<float>::stream(s, indent + "  ", v.amp_temp);
    s << indent << "current: ";
    Printer<float>::stream(s, indent + "  ", v.current);
    s << indent << "torque: ";
    Printer<float>::stream(s, indent + "  ", v.torque);
    s << indent << "torquedot: ";
    Printer<float>::stream(s, indent + "  ", v.torquedot);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "thetadot: ";
    Printer<float>::stream(s, indent + "  ", v.thetadot);
    s << indent << "thetadotdot: ";
    Printer<float>::stream(s, indent + "  ", v.thetadotdot);
    s << indent << "torque_gravity: ";
    Printer<float>::stream(s, indent + "  ", v.torque_gravity);
    s << indent << "pwm_cmd: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pwm_cmd);
    s << indent << "ambient_temp: ";
    Printer<float>::stream(s, indent + "  ", v.ambient_temp);
    s << indent << "case_temp: ";
    Printer<float>::stream(s, indent + "  ", v.case_temp);
    s << indent << "power: ";
    Printer<float>::stream(s, indent + "  ", v.power);
    s << indent << "flags: ";
    Printer<int32_t>::stream(s, indent + "  ", v.flags);
  }
};


} // namespace message_operations
} // namespace ros

#endif // M3MEKA_MSGS_MESSAGE_M3JOINTSTATUS_H

