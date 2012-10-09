/* Auto-generated by genmsg_cpp for file /opt/ros/fuerte/stacks/ardrone_autonomy/srv/LedAnim.srv */
#ifndef ARDRONE_AUTONOMY_SERVICE_LEDANIM_H
#define ARDRONE_AUTONOMY_SERVICE_LEDANIM_H
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




namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct LedAnimRequest_ {
  typedef LedAnimRequest_<ContainerAllocator> Type;

  LedAnimRequest_()
  : type(0)
  , freq(0.0)
  , duration(0)
  {
  }

  LedAnimRequest_(const ContainerAllocator& _alloc)
  : type(0)
  , freq(0.0)
  , duration(0)
  {
  }

  typedef uint8_t _type_type;
  uint8_t type;

  typedef float _freq_type;
  float freq;

  typedef uint8_t _duration_type;
  uint8_t duration;


  typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LedAnimRequest
typedef  ::ardrone_autonomy::LedAnimRequest_<std::allocator<void> > LedAnimRequest;

typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimRequest> LedAnimRequestPtr;
typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimRequest const> LedAnimRequestConstPtr;


template <class ContainerAllocator>
struct LedAnimResponse_ {
  typedef LedAnimResponse_<ContainerAllocator> Type;

  LedAnimResponse_()
  : result(false)
  {
  }

  LedAnimResponse_(const ContainerAllocator& _alloc)
  : result(false)
  {
  }

  typedef uint8_t _result_type;
  uint8_t result;


  typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LedAnimResponse
typedef  ::ardrone_autonomy::LedAnimResponse_<std::allocator<void> > LedAnimResponse;

typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimResponse> LedAnimResponsePtr;
typedef boost::shared_ptr< ::ardrone_autonomy::LedAnimResponse const> LedAnimResponseConstPtr;

struct LedAnim
{

typedef LedAnimRequest Request;
typedef LedAnimResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct LedAnim
} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "23392fc8200b12a3585ff6a32d597821";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x23392fc8200b12a3ULL;
  static const uint64_t static_value2 = 0x585ff6a32d597821ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_autonomy/LedAnimRequest";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
uint8 type\n\
\n\
\n\
float32 freq\n\
\n\
\n\
uint8 duration\n\
\n\
\n\
";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_autonomy/LedAnimResponse";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool result\n\
\n\
\n\
";
  }

  static const char* value(const  ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::LedAnimRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.type);
    stream.next(m.freq);
    stream.next(m.duration);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LedAnimRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::LedAnimResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LedAnimResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<ardrone_autonomy::LedAnim> {
  static const char* value() 
  {
    return "0447d1620f8ba70a5b1fc2d89e406549";
  }

  static const char* value(const ardrone_autonomy::LedAnim&) { return value(); } 
};

template<>
struct DataType<ardrone_autonomy::LedAnim> {
  static const char* value() 
  {
    return "ardrone_autonomy/LedAnim";
  }

  static const char* value(const ardrone_autonomy::LedAnim&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0447d1620f8ba70a5b1fc2d89e406549";
  }

  static const char* value(const ardrone_autonomy::LedAnimRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ardrone_autonomy::LedAnimRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_autonomy/LedAnim";
  }

  static const char* value(const ardrone_autonomy::LedAnimRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0447d1620f8ba70a5b1fc2d89e406549";
  }

  static const char* value(const ardrone_autonomy::LedAnimResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ardrone_autonomy::LedAnimResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_autonomy/LedAnim";
  }

  static const char* value(const ardrone_autonomy::LedAnimResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ARDRONE_AUTONOMY_SERVICE_LEDANIM_H

