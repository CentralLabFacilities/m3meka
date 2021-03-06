/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka/ros/shm_led_mouth/msg/LEDMatrixRow.msg */
#ifndef SHM_LED_MOUTH_MESSAGE_LEDMATRIXROW_H
#define SHM_LED_MOUTH_MESSAGE_LEDMATRIXROW_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "shm_led_mouth/LEDMatrixRGB.h"

namespace shm_led_mouth
{
template <class ContainerAllocator>
struct LEDMatrixRow_ {
  typedef LEDMatrixRow_<ContainerAllocator> Type;

  LEDMatrixRow_()
  : column()
  {
  }

  LEDMatrixRow_(const ContainerAllocator& _alloc)
  : column(_alloc)
  {
  }

  typedef std::vector< ::shm_led_mouth::LEDMatrixRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_mouth::LEDMatrixRGB_<ContainerAllocator> >::other >  _column_type;
  std::vector< ::shm_led_mouth::LEDMatrixRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_mouth::LEDMatrixRGB_<ContainerAllocator> >::other >  column;


  typedef boost::shared_ptr< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LEDMatrixRow
typedef  ::shm_led_mouth::LEDMatrixRow_<std::allocator<void> > LEDMatrixRow;

typedef boost::shared_ptr< ::shm_led_mouth::LEDMatrixRow> LEDMatrixRowPtr;
typedef boost::shared_ptr< ::shm_led_mouth::LEDMatrixRow const> LEDMatrixRowConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace shm_led_mouth

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5a0313057dae76e530e4ef99ceb0eca4";
  }

  static const char* value(const  ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5a0313057dae76e5ULL;
  static const uint64_t static_value2 = 0x30e4ef99ceb0eca4ULL;
};

template<class ContainerAllocator>
struct DataType< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> > {
  static const char* value() 
  {
    return "shm_led_mouth/LEDMatrixRow";
  }

  static const char* value(const  ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> > {
  static const char* value() 
  {
    return "LEDMatrixRGB[] column\n\
================================================================================\n\
MSG: shm_led_mouth/LEDMatrixRGB\n\
uint32 r\n\
uint32 g\n\
uint32 b\n\
";
  }

  static const char* value(const  ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.column);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LEDMatrixRow_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::shm_led_mouth::LEDMatrixRow_<ContainerAllocator> & v) 
  {
    s << indent << "column[]" << std::endl;
    for (size_t i = 0; i < v.column.size(); ++i)
    {
      s << indent << "  column[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::shm_led_mouth::LEDMatrixRGB_<ContainerAllocator> >::stream(s, indent + "    ", v.column[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SHM_LED_MOUTH_MESSAGE_LEDMATRIXROW_H

