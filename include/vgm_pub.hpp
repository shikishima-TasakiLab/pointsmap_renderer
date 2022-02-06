#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointsmap_renderer/VoxelGridMap.h"
#include "voxel_grid_map.hpp"

#define DEFAULT_MAP_FRAMEID "map"

class VGM_Pub
{
public:
    VGM_Pub();
private:
    ros::NodeHandlePtr _nh;
    boost::shared_ptr<ros::Subscriber> _sub;
    boost::shared_ptr<ros::Publisher> _pub;
    float_t _voxel_size = DEFAULT_VOXEL_SIZE;

    void _callback(const sensor_msgs::PointCloud2ConstPtr &msg);
};
