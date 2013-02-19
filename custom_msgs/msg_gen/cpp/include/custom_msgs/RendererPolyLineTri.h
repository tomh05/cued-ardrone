/* Auto-generated by genmsg_cpp for file /home/alex/cued-ardrone/custom_msgs/msg/RendererPolyLineTri.msg */
#ifndef CUSTOM_MSGS_MESSAGE_RENDERERPOLYLINETRI_H
#define CUSTOM_MSGS_MESSAGE_RENDERERPOLYLINETRI_H
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

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Polygon.h"

namespace custom_msgs
{
template <class ContainerAllocator>
struct RendererPolyLineTri_ {
  typedef RendererPolyLineTri_<ContainerAllocator> Type;

  RendererPolyLineTri_()
  : floorlines()
  , polygons()
  , triangles()
  {
  }

  RendererPolyLineTri_(const ContainerAllocator& _alloc)
  : floorlines(_alloc)
  , polygons(_alloc)
  , triangles(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _floorlines_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  floorlines;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _polygons_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  polygons;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _triangles_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  triangles;


  typedef boost::shared_ptr< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RendererPolyLineTri
typedef  ::custom_msgs::RendererPolyLineTri_<std::allocator<void> > RendererPolyLineTri;

typedef boost::shared_ptr< ::custom_msgs::RendererPolyLineTri> RendererPolyLineTriPtr;
typedef boost::shared_ptr< ::custom_msgs::RendererPolyLineTri const> RendererPolyLineTriConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace custom_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2e629a34699ab4fdf40197c2a9032c96";
  }

  static const char* value(const  ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2e629a34699ab4fdULL;
  static const uint64_t static_value2 = 0xf40197c2a9032c96ULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> > {
  static const char* value() 
  {
    return "custom_msgs/RendererPolyLineTri";
  }

  static const char* value(const  ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] floorlines\n\
geometry_msgs/Polygon[] polygons\n\
geometry_msgs/Polygon[] triangles\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.floorlines);
    stream.next(m.polygons);
    stream.next(m.triangles);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RendererPolyLineTri_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::custom_msgs::RendererPolyLineTri_<ContainerAllocator> & v) 
  {
    s << indent << "floorlines[]" << std::endl;
    for (size_t i = 0; i < v.floorlines.size(); ++i)
    {
      s << indent << "  floorlines[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.floorlines[i]);
    }
    s << indent << "polygons[]" << std::endl;
    for (size_t i = 0; i < v.polygons.size(); ++i)
    {
      s << indent << "  polygons[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.polygons[i]);
    }
    s << indent << "triangles[]" << std::endl;
    for (size_t i = 0; i < v.triangles.size(); ++i)
    {
      s << indent << "  triangles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.triangles[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_RENDERERPOLYLINETRI_H
