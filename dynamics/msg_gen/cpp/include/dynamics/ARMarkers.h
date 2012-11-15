/* Auto-generated by genmsg_cpp for file /home/rujian/fuerte_workspace/cued-ardrone/dynamics/msg/ARMarkers.msg */
#ifndef DYNAMICS_MESSAGE_ARMARKERS_H
#define DYNAMICS_MESSAGE_ARMARKERS_H
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

#include "std_msgs/Header.h"
#include "dynamics/ARMarker.h"

namespace dynamics
{
template <class ContainerAllocator>
struct ARMarkers_ {
  typedef ARMarkers_<ContainerAllocator> Type;

  ARMarkers_()
  : header()
  , markers()
  {
  }

  ARMarkers_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , markers(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::dynamics::ARMarker_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::dynamics::ARMarker_<ContainerAllocator> >::other >  _markers_type;
  std::vector< ::dynamics::ARMarker_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::dynamics::ARMarker_<ContainerAllocator> >::other >  markers;


  typedef boost::shared_ptr< ::dynamics::ARMarkers_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamics::ARMarkers_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ARMarkers
typedef  ::dynamics::ARMarkers_<std::allocator<void> > ARMarkers;

typedef boost::shared_ptr< ::dynamics::ARMarkers> ARMarkersPtr;
typedef boost::shared_ptr< ::dynamics::ARMarkers const> ARMarkersConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::dynamics::ARMarkers_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::dynamics::ARMarkers_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace dynamics

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dynamics::ARMarkers_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dynamics::ARMarkers_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dynamics::ARMarkers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b35e1e178a9cd7039dbb63cf2764131a";
  }

  static const char* value(const  ::dynamics::ARMarkers_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb35e1e178a9cd703ULL;
  static const uint64_t static_value2 = 0x9dbb63cf2764131aULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamics::ARMarkers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dynamics/ARMarkers";
  }

  static const char* value(const  ::dynamics::ARMarkers_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dynamics::ARMarkers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
ARMarker[] markers\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: dynamics/ARMarker\n\
Header header\n\
uint32 id\n\
geometry_msgs/PoseWithCovariance pose\n\
uint32 confidence\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::dynamics::ARMarkers_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::dynamics::ARMarkers_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::dynamics::ARMarkers_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dynamics::ARMarkers_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.markers);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ARMarkers_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamics::ARMarkers_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::dynamics::ARMarkers_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "markers[]" << std::endl;
    for (size_t i = 0; i < v.markers.size(); ++i)
    {
      s << indent << "  markers[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dynamics::ARMarker_<ContainerAllocator> >::stream(s, indent + "    ", v.markers[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // DYNAMICS_MESSAGE_ARMARKERS_H

