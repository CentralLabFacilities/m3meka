/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka-new/ros/m3meka_msgs/msg/M3OmnibaseJoy.msg */
#ifndef M3MEKA_MSGS_MESSAGE_M3OMNIBASEJOY_H
#define M3MEKA_MSGS_MESSAGE_M3OMNIBASEJOY_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace m3meka_msgs
{
template <class ContainerAllocator>
struct M3OmnibaseJoy_ : public ros::Message
{
  typedef M3OmnibaseJoy_<ContainerAllocator> Type;

  M3OmnibaseJoy_()
  : x(0.0)
  , y(0.0)
  , yaw(0.0)
  , button(0.0)
  , z(0.0)
  {
  }

  M3OmnibaseJoy_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , yaw(0.0)
  , button(0.0)
  , z(0.0)
  {
  }

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _yaw_type;
  float yaw;

  typedef float _button_type;
  float button;

  typedef float _z_type;
  float z;


private:
  static const char* __s_getDataType_() { return "m3meka_msgs/M3OmnibaseJoy"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "6719502035b93742f7b2585c261584a9"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 x\n\
float32 y\n\
float32 yaw\n\
float32 button\n\
float32 z\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, yaw);
    ros::serialization::serialize(stream, button);
    ros::serialization::serialize(stream, z);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, yaw);
    ros::serialization::deserialize(stream, button);
    ros::serialization::deserialize(stream, z);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(yaw);
    size += ros::serialization::serializationLength(button);
    size += ros::serialization::serializationLength(z);
    return size;
  }

  typedef boost::shared_ptr< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator>  const> ConstPtr;
}; // struct M3OmnibaseJoy
typedef  ::m3meka_msgs::M3OmnibaseJoy_<std::allocator<void> > M3OmnibaseJoy;

typedef boost::shared_ptr< ::m3meka_msgs::M3OmnibaseJoy> M3OmnibaseJoyPtr;
typedef boost::shared_ptr< ::m3meka_msgs::M3OmnibaseJoy const> M3OmnibaseJoyConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace m3meka_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6719502035b93742f7b2585c261584a9";
  }

  static const char* value(const  ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6719502035b93742ULL;
  static const uint64_t static_value2 = 0xf7b2585c261584a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3OmnibaseJoy";
  }

  static const char* value(const  ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 x\n\
float32 y\n\
float32 yaw\n\
float32 button\n\
float32 z\n\
\n\
\n\
";
  }

  static const char* value(const  ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.yaw);
    stream.next(m.button);
    stream.next(m.z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3OmnibaseJoy_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::m3meka_msgs::M3OmnibaseJoy_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "button: ";
    Printer<float>::stream(s, indent + "  ", v.button);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // M3MEKA_MSGS_MESSAGE_M3OMNIBASEJOY_H

