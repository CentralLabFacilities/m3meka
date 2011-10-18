/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka-new/ros/m3meka_msgs/srv/M3LoadX6Param.srv */
#ifndef M3MEKA_MSGS_SERVICE_M3LOADX6PARAM_H
#define M3MEKA_MSGS_SERVICE_M3LOADX6PARAM_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace m3meka_msgs
{
template <class ContainerAllocator>
struct M3LoadX6ParamRequest_ : public ros::Message
{
  typedef M3LoadX6ParamRequest_<ContainerAllocator> Type;

  M3LoadX6ParamRequest_()
  : request(0)
  {
  }

  M3LoadX6ParamRequest_(const ContainerAllocator& _alloc)
  : request(0)
  {
  }

  typedef int32_t _request_type;
  int32_t request;


private:
  static const char* __s_getDataType_() { return "m3meka_msgs/M3LoadX6ParamRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "650f0ccd41c8f8d53ada80be6ddde434"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "51edd9dfd50014fde2b589cbf77706aa"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 request\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, request);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, request);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(request);
    return size;
  }

  typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct M3LoadX6ParamRequest
typedef  ::m3meka_msgs::M3LoadX6ParamRequest_<std::allocator<void> > M3LoadX6ParamRequest;

typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamRequest> M3LoadX6ParamRequestPtr;
typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamRequest const> M3LoadX6ParamRequestConstPtr;


template <class ContainerAllocator>
struct M3LoadX6ParamResponse_ : public ros::Message
{
  typedef M3LoadX6ParamResponse_<ContainerAllocator> Type;

  M3LoadX6ParamResponse_()
  : response(0)
  {
  }

  M3LoadX6ParamResponse_(const ContainerAllocator& _alloc)
  : response(0)
  {
  }

  typedef int32_t _response_type;
  int32_t response;


private:
  static const char* __s_getDataType_() { return "m3meka_msgs/M3LoadX6ParamResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "f45f68e2feefb1307598e828e260b7d7"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "51edd9dfd50014fde2b589cbf77706aa"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 response\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, response);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, response);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(response);
    return size;
  }

  typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct M3LoadX6ParamResponse
typedef  ::m3meka_msgs::M3LoadX6ParamResponse_<std::allocator<void> > M3LoadX6ParamResponse;

typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamResponse> M3LoadX6ParamResponsePtr;
typedef boost::shared_ptr< ::m3meka_msgs::M3LoadX6ParamResponse const> M3LoadX6ParamResponseConstPtr;

struct M3LoadX6Param
{

typedef M3LoadX6ParamRequest Request;
typedef M3LoadX6ParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct M3LoadX6Param
} // namespace m3meka_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "650f0ccd41c8f8d53ada80be6ddde434";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x650f0ccd41c8f8d5ULL;
  static const uint64_t static_value2 = 0x3ada80be6ddde434ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3LoadX6ParamRequest";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 request\n\
\n\
";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f45f68e2feefb1307598e828e260b7d7";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf45f68e2feefb130ULL;
  static const uint64_t static_value2 = 0x7598e828e260b7d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3LoadX6ParamResponse";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 response\n\
\n\
";
  }

  static const char* value(const  ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.request);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3LoadX6ParamRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.response);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct M3LoadX6ParamResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<m3meka_msgs::M3LoadX6Param> {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3meka_msgs::M3LoadX6Param&) { return value(); } 
};

template<>
struct DataType<m3meka_msgs::M3LoadX6Param> {
  static const char* value() 
  {
    return "m3meka_msgs/M3LoadX6Param";
  }

  static const char* value(const m3meka_msgs::M3LoadX6Param&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3LoadX6Param";
  }

  static const char* value(const m3meka_msgs::M3LoadX6ParamRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "51edd9dfd50014fde2b589cbf77706aa";
  }

  static const char* value(const m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "m3meka_msgs/M3LoadX6Param";
  }

  static const char* value(const m3meka_msgs::M3LoadX6ParamResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // M3MEKA_MSGS_SERVICE_M3LOADX6PARAM_H

