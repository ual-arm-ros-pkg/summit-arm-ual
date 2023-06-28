/* Auto-generated by genmsg_cpp for file
 * /home/guardian-wam/Sources/gwam-ros-pkg/guardian/guardian_robot/sphereptz/msg/ptz_state.msg
 */
#ifndef SPHEREPTZ_MESSAGE_PTZ_STATE_H
#define SPHEREPTZ_MESSAGE_PTZ_STATE_H
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "ros/assert.h"
#include "ros/builtin_message_traits.h"
#include "ros/macros.h"
#include "ros/message_operations.h"
#include "ros/serialization.h"
#include "ros/time.h"

namespace sphereptz
{
template <class ContainerAllocator>
struct ptz_state_
{
    typedef ptz_state_<ContainerAllocator> Type;

    ptz_state_() : pan(0.0), tilt(0.0), zoom(0.0) {}

    ptz_state_(const ContainerAllocator& _alloc)
        : pan(0.0), tilt(0.0), zoom(0.0)
    {
    }

    typedef float _pan_type;
    float         pan;

    typedef float _tilt_type;
    float         tilt;

    typedef float _zoom_type;
    float         zoom;

   private:
    static const char* __s_getDataType_() { return "sphereptz/ptz_state"; }

   public:
    ROS_DEPRECATED static const std::string __s_getDataType()
    {
        return __s_getDataType_();
    }

    ROS_DEPRECATED const std::string __getDataType() const
    {
        return __s_getDataType_();
    }

   private:
    static const char* __s_getMD5Sum_()
    {
        return "eef4827ef5b8ba89a8c9fa0810a30294";
    }

   public:
    ROS_DEPRECATED static const std::string __s_getMD5Sum()
    {
        return __s_getMD5Sum_();
    }

    ROS_DEPRECATED const std::string __getMD5Sum() const
    {
        return __s_getMD5Sum_();
    }

   private:
    static const char* __s_getMessageDefinition_()
    {
        return "# Pan-tilt positions in [rad] zoom = -1 as Logitech Quickcam Sphere has only focus, not zoom\n\
float32 pan\n\
float32 tilt\n\
float32 zoom\n\
\n\
\n\
";
    }

   public:
    ROS_DEPRECATED static const std::string __s_getMessageDefinition()
    {
        return __s_getMessageDefinition_();
    }

    ROS_DEPRECATED const std::string __getMessageDefinition() const
    {
        return __s_getMessageDefinition_();
    }

    ROS_DEPRECATED virtual uint8_t* serialize(
        uint8_t* write_ptr, uint32_t seq) const
    {
        ros::serialization::OStream stream(write_ptr, 1000000000);
        ros::serialization::serialize(stream, pan);
        ros::serialization::serialize(stream, tilt);
        ros::serialization::serialize(stream, zoom);
        return stream.getData();
    }

    ROS_DEPRECATED virtual uint8_t* deserialize(uint8_t* read_ptr)
    {
        ros::serialization::IStream stream(read_ptr, 1000000000);
        ros::serialization::deserialize(stream, pan);
        ros::serialization::deserialize(stream, tilt);
        ros::serialization::deserialize(stream, zoom);
        return stream.getData();
    }

    ROS_DEPRECATED virtual uint32_t serializationLength() const
    {
        uint32_t size = 0;
        size += ros::serialization::serializationLength(pan);
        size += ros::serialization::serializationLength(tilt);
        size += ros::serialization::serializationLength(zoom);
        return size;
    }

    typedef boost::shared_ptr<::sphereptz::ptz_state_<ContainerAllocator>> Ptr;
    typedef boost::shared_ptr<::sphereptz::ptz_state_<ContainerAllocator> const>
                                                          ConstPtr;
    boost::shared_ptr<std::map<std::string, std::string>> __connection_header;
};  // struct ptz_state
typedef ::sphereptz::ptz_state_<std::allocator<void>> ptz_state;

typedef boost::shared_ptr<::sphereptz::ptz_state>       ptz_statePtr;
typedef boost::shared_ptr<::sphereptz::ptz_state const> ptz_stateConstPtr;

template <typename ContainerAllocator>
std::ostream& operator<<(
    std::ostream& s, const ::sphereptz::ptz_state_<ContainerAllocator>& v)
{
    ros::message_operations::Printer<
        ::sphereptz::ptz_state_<ContainerAllocator>>::stream(s, "", v);
    return s;
}

}  // namespace sphereptz

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage<::sphereptz::ptz_state_<ContainerAllocator>> : public TrueType
{
};
template <class ContainerAllocator>
struct IsMessage<::sphereptz::ptz_state_<ContainerAllocator> const>
    : public TrueType
{
};
template <class ContainerAllocator>
struct MD5Sum<::sphereptz::ptz_state_<ContainerAllocator>>
{
    static const char* value() { return "eef4827ef5b8ba89a8c9fa0810a30294"; }

    static const char* value(const ::sphereptz::ptz_state_<ContainerAllocator>&)
    {
        return value();
    }
    static const uint64_t static_value1 = 0xeef4827ef5b8ba89ULL;
    static const uint64_t static_value2 = 0xa8c9fa0810a30294ULL;
};

template <class ContainerAllocator>
struct DataType<::sphereptz::ptz_state_<ContainerAllocator>>
{
    static const char* value() { return "sphereptz/ptz_state"; }

    static const char* value(const ::sphereptz::ptz_state_<ContainerAllocator>&)
    {
        return value();
    }
};

template <class ContainerAllocator>
struct Definition<::sphereptz::ptz_state_<ContainerAllocator>>
{
    static const char* value()
    {
        return "# Pan-tilt positions in [rad] zoom = -1 as Logitech Quickcam Sphere has only focus, not zoom\n\
float32 pan\n\
float32 tilt\n\
float32 zoom\n\
\n\
\n\
";
    }

    static const char* value(const ::sphereptz::ptz_state_<ContainerAllocator>&)
    {
        return value();
    }
};

template <class ContainerAllocator>
struct IsFixedSize<::sphereptz::ptz_state_<ContainerAllocator>>
    : public TrueType
{
};
}  // namespace message_traits
}  // namespace ros

namespace ros
{
namespace serialization
{
template <class ContainerAllocator>
struct Serializer<::sphereptz::ptz_state_<ContainerAllocator>>
{
    template <typename Stream, typename T>
    inline static void allInOne(Stream& stream, T m)
    {
        stream.next(m.pan);
        stream.next(m.tilt);
        stream.next(m.zoom);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
};  // struct ptz_state_
}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <class ContainerAllocator>
struct Printer<::sphereptz::ptz_state_<ContainerAllocator>>
{
    template <typename Stream>
    static void stream(
        Stream& s, const std::string& indent,
        const ::sphereptz::ptz_state_<ContainerAllocator>& v)
    {
        s << indent << "pan: ";
        Printer<float>::stream(s, indent + "  ", v.pan);
        s << indent << "tilt: ";
        Printer<float>::stream(s, indent + "  ", v.tilt);
        s << indent << "zoom: ";
        Printer<float>::stream(s, indent + "  ", v.zoom);
    }
};

}  // namespace message_operations
}  // namespace ros

#endif  // SPHEREPTZ_MESSAGE_PTZ_STATE_H
