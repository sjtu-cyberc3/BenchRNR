// Generated by gencpp from file cyber_msgs/Heading.msg
// DO NOT EDIT!

/*
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 data
float64 std_dev
*/


#ifndef CYBER_MSGS_MESSAGE_HEADING_H
#define CYBER_MSGS_MESSAGE_HEADING_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatStatus.h>

namespace cyber_msgs
{
template <class ContainerAllocator>
struct Heading_
{
  typedef Heading_<ContainerAllocator> Type;

  Heading_()
    : header()
    , status()
    , data(0.0)
    , std_dev(0.0)  {
    }
  Heading_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , data(0.0)
    , std_dev(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::sensor_msgs::NavSatStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef double _data_type;
  _data_type data;

   typedef double _std_dev_type;
  _std_dev_type std_dev;





  typedef boost::shared_ptr< ::cyber_msgs::Heading_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cyber_msgs::Heading_<ContainerAllocator> const> ConstPtr;

}; // struct Heading_

typedef ::cyber_msgs::Heading_<std::allocator<void> > Heading;

typedef boost::shared_ptr< ::cyber_msgs::Heading > HeadingPtr;
typedef boost::shared_ptr< ::cyber_msgs::Heading const> HeadingConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cyber_msgs::Heading_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cyber_msgs::Heading_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cyber_msgs::Heading_<ContainerAllocator1> & lhs, const ::cyber_msgs::Heading_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.status == rhs.status &&
    lhs.data == rhs.data &&
    lhs.std_dev == rhs.std_dev;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cyber_msgs::Heading_<ContainerAllocator1> & lhs, const ::cyber_msgs::Heading_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cyber_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cyber_msgs::Heading_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cyber_msgs::Heading_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_msgs::Heading_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyber_msgs::Heading_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_msgs::Heading_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyber_msgs::Heading_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cyber_msgs::Heading_<ContainerAllocator> >
{
  static const char* value()
  {
    return "908b85f3e12fe5f8bf9abe03aeb2ef3f";
  }

  static const char* value(const ::cyber_msgs::Heading_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x908b85f3e12fe5f8ULL;
  static const uint64_t static_value2 = 0xbf9abe03aeb2ef3fULL;
};

template<class ContainerAllocator>
struct DataType< ::cyber_msgs::Heading_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cyber_msgs/Heading";
  }

  static const char* value(const ::cyber_msgs::Heading_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cyber_msgs::Heading_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"sensor_msgs/NavSatStatus status\n"
"float64 data\n"
"float64 std_dev\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/NavSatStatus\n"
"# Navigation Satellite fix status for any Global Navigation Satellite System\n"
"\n"
"# Whether to output an augmented fix is determined by both the fix\n"
"# type and the last time differential corrections were received.  A\n"
"# fix is valid when status >= STATUS_FIX.\n"
"\n"
"int8 STATUS_NO_FIX =  -1        # unable to fix position\n"
"int8 STATUS_FIX =      0        # unaugmented fix\n"
"int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\n"
"int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n"
"\n"
"int8 status\n"
"\n"
"# Bits defining which Global Navigation Satellite System signals were\n"
"# used by the receiver.\n"
"\n"
"uint16 SERVICE_GPS =     1\n"
"uint16 SERVICE_GLONASS = 2\n"
"uint16 SERVICE_COMPASS = 4      # includes BeiDou.\n"
"uint16 SERVICE_GALILEO = 8\n"
"\n"
"uint16 service\n"
;
  }

  static const char* value(const ::cyber_msgs::Heading_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cyber_msgs::Heading_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.data);
      stream.next(m.std_dev);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Heading_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cyber_msgs::Heading_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cyber_msgs::Heading_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::sensor_msgs::NavSatStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "data: ";
    Printer<double>::stream(s, indent + "  ", v.data);
    s << indent << "std_dev: ";
    Printer<double>::stream(s, indent + "  ", v.std_dev);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CYBER_MSGS_MESSAGE_HEADING_H
