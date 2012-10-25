/* Auto-generated by genmsg_cpp for file /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/CamSelect.srv */
#ifndef DYNAMICS_SERVICE_CAMSELECT_H
#define DYNAMICS_SERVICE_CAMSELECT_H
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




namespace dynamics
{
template <class ContainerAllocator>
struct CamSelectRequest_ {
  typedef CamSelectRequest_<ContainerAllocator> Type;

  CamSelectRequest_()
  : channel(0)
  {
  }

  CamSelectRequest_(const ContainerAllocator& _alloc)
  : channel(0)
  {
  }

  typedef uint8_t _channel_type;
  uint8_t channel;


  typedef boost::shared_ptr< ::dynamics::CamSelectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamics::CamSelectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CamSelectRequest
typedef  ::dynamics::CamSelectRequest_<std::allocator<void> > CamSelectRequest;

typedef boost::shared_ptr< ::dynamics::CamSelectRequest> CamSelectRequestPtr;
typedef boost::shared_ptr< ::dynamics::CamSelectRequest const> CamSelectRequestConstPtr;


template <class ContainerAllocator>
struct CamSelectResponse_ {
  typedef CamSelectResponse_<ContainerAllocator> Type;

  CamSelectResponse_()
  : result(false)
  {
  }

  CamSelectResponse_(const ContainerAllocator& _alloc)
  : result(false)
  {
  }

  typedef uint8_t _result_type;
  uint8_t result;


  typedef boost::shared_ptr< ::dynamics::CamSelectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamics::CamSelectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CamSelectResponse
typedef  ::dynamics::CamSelectResponse_<std::allocator<void> > CamSelectResponse;

typedef boost::shared_ptr< ::dynamics::CamSelectResponse> CamSelectResponsePtr;
typedef boost::shared_ptr< ::dynamics::CamSelectResponse const> CamSelectResponseConstPtr;

struct CamSelect
{

typedef CamSelectRequest Request;
typedef CamSelectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CamSelect
} // namespace dynamics

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dynamics::CamSelectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dynamics::CamSelectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dynamics::CamSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c27320df100593b008f1bb2e1302dbb6";
  }

  static const char* value(const  ::dynamics::CamSelectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc27320df100593b0ULL;
  static const uint64_t static_value2 = 0x08f1bb2e1302dbb6ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamics::CamSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamics/CamSelectRequest";
  }

  static const char* value(const  ::dynamics::CamSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dynamics::CamSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 channel\n\
\n\
";
  }

  static const char* value(const  ::dynamics::CamSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dynamics::CamSelectRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dynamics::CamSelectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dynamics::CamSelectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dynamics::CamSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const  ::dynamics::CamSelectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamics::CamSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamics/CamSelectResponse";
  }

  static const char* value(const  ::dynamics::CamSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dynamics::CamSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool result\n\
\n\
\n\
";
  }

  static const char* value(const  ::dynamics::CamSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::dynamics::CamSelectResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dynamics::CamSelectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.channel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CamSelectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dynamics::CamSelectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CamSelectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<dynamics::CamSelect> {
  static const char* value() 
  {
    return "bbeb5212f8ee1d6da7ff0d1169124280";
  }

  static const char* value(const dynamics::CamSelect&) { return value(); } 
};

template<>
struct DataType<dynamics::CamSelect> {
  static const char* value() 
  {
    return "dynamics/CamSelect";
  }

  static const char* value(const dynamics::CamSelect&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dynamics::CamSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bbeb5212f8ee1d6da7ff0d1169124280";
  }

  static const char* value(const dynamics::CamSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dynamics::CamSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamics/CamSelect";
  }

  static const char* value(const dynamics::CamSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<dynamics::CamSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bbeb5212f8ee1d6da7ff0d1169124280";
  }

  static const char* value(const dynamics::CamSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<dynamics::CamSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamics/CamSelect";
  }

  static const char* value(const dynamics::CamSelectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMICS_SERVICE_CAMSELECT_H
