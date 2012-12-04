/* Auto-generated by genmsg_cpp for file /home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_gui/srv/SolutionSelect.srv */
#ifndef FHWA2_GUI_SERVICE_SOLUTIONSELECT_H
#define FHWA2_GUI_SERVICE_SOLUTIONSELECT_H
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




namespace fhwa2_gui
{
template <class ContainerAllocator>
struct SolutionSelectRequest_ {
  typedef SolutionSelectRequest_<ContainerAllocator> Type;

  SolutionSelectRequest_()
  : target()
  {
  }

  SolutionSelectRequest_(const ContainerAllocator& _alloc)
  : target(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _target_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  target;


  typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SolutionSelectRequest
typedef  ::fhwa2_gui::SolutionSelectRequest_<std::allocator<void> > SolutionSelectRequest;

typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectRequest> SolutionSelectRequestPtr;
typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectRequest const> SolutionSelectRequestConstPtr;


template <class ContainerAllocator>
struct SolutionSelectResponse_ {
  typedef SolutionSelectResponse_<ContainerAllocator> Type;

  SolutionSelectResponse_()
  : sucess(false)
  {
  }

  SolutionSelectResponse_(const ContainerAllocator& _alloc)
  : sucess(false)
  {
  }

  typedef uint8_t _sucess_type;
  uint8_t sucess;


  typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SolutionSelectResponse
typedef  ::fhwa2_gui::SolutionSelectResponse_<std::allocator<void> > SolutionSelectResponse;

typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectResponse> SolutionSelectResponsePtr;
typedef boost::shared_ptr< ::fhwa2_gui::SolutionSelectResponse const> SolutionSelectResponseConstPtr;

struct SolutionSelect
{

typedef SolutionSelectRequest Request;
typedef SolutionSelectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SolutionSelect
} // namespace fhwa2_gui

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "10e5cb524446adda5ea1765c6838590d";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x10e5cb524446addaULL;
  static const uint64_t static_value2 = 0x5ea1765c6838590dULL;
};

template<class ContainerAllocator>
struct DataType< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fhwa2_gui/SolutionSelectRequest";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
string target\n\
\n\
";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3e8ad2f62d0edf01ac96b1e92459b490";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3e8ad2f62d0edf01ULL;
  static const uint64_t static_value2 = 0xac96b1e92459b490ULL;
};

template<class ContainerAllocator>
struct DataType< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fhwa2_gui/SolutionSelectResponse";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool sucess\n\
\n\
";
  }

  static const char* value(const  ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.target);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SolutionSelectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.sucess);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SolutionSelectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<fhwa2_gui::SolutionSelect> {
  static const char* value() 
  {
    return "ec12f023325f9f4d99019044ac32e2c0";
  }

  static const char* value(const fhwa2_gui::SolutionSelect&) { return value(); } 
};

template<>
struct DataType<fhwa2_gui::SolutionSelect> {
  static const char* value() 
  {
    return "fhwa2_gui/SolutionSelect";
  }

  static const char* value(const fhwa2_gui::SolutionSelect&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ec12f023325f9f4d99019044ac32e2c0";
  }

  static const char* value(const fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fhwa2_gui/SolutionSelect";
  }

  static const char* value(const fhwa2_gui::SolutionSelectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ec12f023325f9f4d99019044ac32e2c0";
  }

  static const char* value(const fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fhwa2_gui/SolutionSelect";
  }

  static const char* value(const fhwa2_gui::SolutionSelectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // FHWA2_GUI_SERVICE_SOLUTIONSELECT_H

