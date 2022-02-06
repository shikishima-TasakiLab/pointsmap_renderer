#pragma once
#include <chrono>
#include "subscriber.hpp"
#include "voxel_grid_map.hpp"
#include "pointsmap_renderer/VoxelGridMap.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"

#define DEFAULT_HZ 5.0
#define DEFAULT_QUEUE_SIZE 10
#define DEFAULT_DEPTH_MIN 0.0
#define DEFAULT_DEPTH_MAX INFINITY

#define THROTTLE_PERIOD 30

class Pointsmap_Renderer
{
public:
    Pointsmap_Renderer();
private:
    ros::NodeHandlePtr _nh;
    boost::shared_ptr<ros::Timer> _timer;

    boost::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    boost::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    boost::shared_ptr<ros::Publisher> _depth_pub;
    boost::shared_ptr<Subscriber<sensor_msgs::CameraInfo> > _camerainfo_sub;
    boost::shared_ptr<Subscriber<pointsmap_renderer::VoxelGridMap> > _map_sub;

    Range _depth_range = {.min = DEFAULT_DEPTH_MIN, .max = DEFAULT_DEPTH_MAX};

    void _timer_callback(const ros::TimerEvent &e);
};
