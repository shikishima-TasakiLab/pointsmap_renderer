#include "pointsmap_renderer.hpp"

Pointsmap_Renderer::Pointsmap_Renderer()
{
    ros::NodeHandle pnh("~");

    double_t hz = DEFAULT_HZ;
    pnh.getParam("hz", hz);
    ROS_INFO_STREAM("hz: " << hz);

    int queue_size = DEFAULT_QUEUE_SIZE;
    pnh.getParam("queue_size", queue_size);
    ROS_INFO_STREAM("queue_size: " << queue_size);

    pnh.getParam("depth_min", this->_depth_range.min);
    ROS_INFO_STREAM("depth_min: " << this->_depth_range.min);

    pnh.getParam("depth_max", this->_depth_range.max);
    ROS_INFO_STREAM("depth_max: " << this->_depth_range.max);

    this->_nh.reset(new ros::NodeHandle("~"));

    this->_tf_buffer.reset(new tf2_ros::Buffer());
    this->_tf_listener.reset(new tf2_ros::TransformListener(*this->_tf_buffer));

    this->_depth_pub.reset(new ros::Publisher(this->_nh->advertise<sensor_msgs::Image>("sparse_depth", queue_size)));
    this->_camerainfo_sub.reset(new Subscriber<sensor_msgs::CameraInfo>(*this->_nh, "camera_info", 2U));
    this->_map_sub.reset(new Subscriber<pointsmap_renderer::VoxelGridMap>(*this->_nh, "/voxel_grid_map", 2U));

    this->_timer.reset(new ros::Timer(_nh->createTimer(ros::Duration(1.0 / hz), &Pointsmap_Renderer::_timer_callback, this)));
}

void Pointsmap_Renderer::_timer_callback(const ros::TimerEvent &timer_event)
{
    pointsmap_renderer::VoxelGridMapConstPtr vgm_ptr = this->_map_sub->get_msg();
    if (vgm_ptr == nullptr) {ROS_WARN_THROTTLE(THROTTLE_PERIOD, "No points."); return;}
    if (vgm_ptr->header.frame_id == "") {ROS_WARN_THROTTLE(THROTTLE_PERIOD, "No frame_id (map)."); return;}

    sensor_msgs::CameraInfoConstPtr camerainfo = this->_camerainfo_sub->get_msg();
    if (camerainfo == nullptr) {ROS_WARN_THROTTLE(THROTTLE_PERIOD, "No camera_info."); return;}
    if (camerainfo->header.frame_id == "") {ROS_WARN_THROTTLE(THROTTLE_PERIOD, "No frame_id (camera_info)"); return;}

    geometry_msgs::TransformStamped tf_map2camera;
    try {
        tf_map2camera = this->_tf_buffer->lookupTransform(
            vgm_ptr->header.frame_id,
            camerainfo->header.frame_id,
            ros::Time(0)
        );
    }
    catch (tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(THROTTLE_PERIOD, "[map(" << vgm_ptr->header.frame_id << ") -> camera (" << camerainfo->header.frame_id << ")] Transform Error: " << e.what());
        return;
    }

    VoxelGridMap<pointsmap_renderer::VoxelGridMapConstPtr> vgm(vgm_ptr);

    cv_bridge::CvImage depth;
    vgm.create_depthmap(camerainfo, tf_map2camera, depth.image, this->_depth_range);
    sensor_msgs::Image sparse_depth = *depth.toImageMsg();
    sparse_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sparse_depth.header.stamp = ros::Time::now();
    sparse_depth.header.frame_id = camerainfo->header.frame_id;
    this->_depth_pub->publish(sparse_depth);
}
