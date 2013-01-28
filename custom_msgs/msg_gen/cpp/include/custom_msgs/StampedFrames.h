/* Auto-generated by genmsg_cpp for file /home/alex/cued-ardrone/custom_msgs/msg/StampedFrames.msg */
#ifndef CUSTOM_MSGS_MESSAGE_STAMPEDFRAMES_H
#define CUSTOM_MSGS_MESSAGE_STAMPEDFRAMES_H
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
#include "custom_msgs/StampedMatchesWithImage.h"
#include "custom_msgs/StampedMatchesWithImage.h"

namespace custom_msgs
{
template <class ContainerAllocator>
struct StampedFrames_ {
  typedef StampedFrames_<ContainerAllocator> Type;

  StampedFrames_()
  : header()
  , frame1()
  , frame2()
  {
  }

  StampedFrames_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , frame1(_alloc)
  , frame2(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator>  _frame1_type;
   ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator>  frame1;

  typedef  ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator>  _frame2_type;
   ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator>  frame2;


  typedef boost::shared_ptr< ::custom_msgs::StampedFrames_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::StampedFrames_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct StampedFrames
typedef  ::custom_msgs::StampedFrames_<std::allocator<void> > StampedFrames;

typedef boost::shared_ptr< ::custom_msgs::StampedFrames> StampedFramesPtr;
typedef boost::shared_ptr< ::custom_msgs::StampedFrames const> StampedFramesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::custom_msgs::StampedFrames_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::custom_msgs::StampedFrames_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace custom_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::custom_msgs::StampedFrames_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::custom_msgs::StampedFrames_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::StampedFrames_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee218eba4fa8a4f54760afca804eb3b2";
  }

  static const char* value(const  ::custom_msgs::StampedFrames_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xee218eba4fa8a4f5ULL;
  static const uint64_t static_value2 = 0x4760afca804eb3b2ULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::StampedFrames_<ContainerAllocator> > {
  static const char* value() 
  {
    return "custom_msgs/StampedFrames";
  }

  static const char* value(const  ::custom_msgs::StampedFrames_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::StampedFrames_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
custom_msgs/StampedMatchesWithImage frame1\n\
custom_msgs/StampedMatchesWithImage frame2\n\
\n\
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
MSG: custom_msgs/StampedMatchesWithImage\n\
Header header\n\
float32[] pts\n\
sensor_msgs/Image image\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in src/image_encodings.cpp\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
";
  }

  static const char* value(const  ::custom_msgs::StampedFrames_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::custom_msgs::StampedFrames_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::custom_msgs::StampedFrames_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::custom_msgs::StampedFrames_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.frame1);
    stream.next(m.frame2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StampedFrames_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::StampedFrames_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::custom_msgs::StampedFrames_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "frame1: ";
s << std::endl;
    Printer< ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator> >::stream(s, indent + "  ", v.frame1);
    s << indent << "frame2: ";
s << std::endl;
    Printer< ::custom_msgs::StampedMatchesWithImage_<ContainerAllocator> >::stream(s, indent + "  ", v.frame2);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_STAMPEDFRAMES_H

