/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/srv/ResetFinger.srv */
#ifndef OWD_MSGS_SERVICE_RESETFINGER_H
#define OWD_MSGS_SERVICE_RESETFINGER_H
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

#include "ros/service_traits.h"




namespace owd_msgs
{
template <class ContainerAllocator>
struct ResetFingerRequest_ {
  typedef ResetFingerRequest_<ContainerAllocator> Type;

  ResetFingerRequest_()
  : finger(0)
  {
  }

  ResetFingerRequest_(const ContainerAllocator& _alloc)
  : finger(0)
  {
  }

  typedef int8_t _finger_type;
  int8_t finger;


  typedef boost::shared_ptr< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::ResetFingerRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResetFingerRequest
typedef  ::owd_msgs::ResetFingerRequest_<std::allocator<void> > ResetFingerRequest;

typedef boost::shared_ptr< ::owd_msgs::ResetFingerRequest> ResetFingerRequestPtr;
typedef boost::shared_ptr< ::owd_msgs::ResetFingerRequest const> ResetFingerRequestConstPtr;



template <class ContainerAllocator>
struct ResetFingerResponse_ {
  typedef ResetFingerResponse_<ContainerAllocator> Type;

  ResetFingerResponse_()
  : ok(false)
  , reason()
  {
  }

  ResetFingerResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::ResetFingerResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResetFingerResponse
typedef  ::owd_msgs::ResetFingerResponse_<std::allocator<void> > ResetFingerResponse;

typedef boost::shared_ptr< ::owd_msgs::ResetFingerResponse> ResetFingerResponsePtr;
typedef boost::shared_ptr< ::owd_msgs::ResetFingerResponse const> ResetFingerResponseConstPtr;


struct ResetFinger
{

typedef ResetFingerRequest Request;
typedef ResetFingerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct ResetFinger
} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetFingerRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "80f9f083fe71012318271dc6232e9766";
  }

  static const char* value(const  ::owd_msgs::ResetFingerRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x80f9f083fe710123ULL;
  static const uint64_t static_value2 = 0x18271dc6232e9766ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetFingerRequest";
  }

  static const char* value(const  ::owd_msgs::ResetFingerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 finger\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::ResetFingerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetFingerResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::owd_msgs::ResetFingerResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetFingerResponse";
  }

  static const char* value(const  ::owd_msgs::ResetFingerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::ResetFingerResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::ResetFingerRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.finger);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetFingerRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::ResetFingerResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetFingerResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<owd_msgs::ResetFinger> {
  static const char* value() 
  {
    return "0e29cc56e438836f4dcc31646927be30";
  }

  static const char* value(const owd_msgs::ResetFinger&) { return value(); } 
};

template<>
struct DataType<owd_msgs::ResetFinger> {
  static const char* value() 
  {
    return "owd_msgs/ResetFinger";
  }

  static const char* value(const owd_msgs::ResetFinger&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::ResetFingerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0e29cc56e438836f4dcc31646927be30";
  }

  static const char* value(const owd_msgs::ResetFingerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::ResetFingerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetFinger";
  }

  static const char* value(const owd_msgs::ResetFingerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::ResetFingerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0e29cc56e438836f4dcc31646927be30";
  }

  static const char* value(const owd_msgs::ResetFingerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::ResetFingerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetFinger";
  }

  static const char* value(const owd_msgs::ResetFingerResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OWD_MSGS_SERVICE_RESETFINGER_H
