#ifndef LIVOX_ROS_DRIVER_CUSTOMMSG_H
#define LIVOX_ROS_DRIVER_CUSTOMMSG_H

#include <std_msgs/Header.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace livox_ros_driver {

struct CustomPoint {
    float x, y, z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;
};

struct CustomMsg {
    typedef boost::shared_ptr<CustomMsg> Ptr;
    typedef boost::shared_ptr<CustomMsg const> ConstPtr;
    
    std_msgs::Header header;
    uint64_t timebase;
    uint32_t point_num;
    uint8_t lidar_id;
    uint8_t rsvd[3];
    std::vector<CustomPoint> points;
};

} // namespace livox_ros_driver

#endif // LIVOX_ROS_DRIVER_CUSTOMMSG_H