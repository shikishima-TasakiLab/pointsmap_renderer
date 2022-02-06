#pragma once
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/opencv.hpp>
#include "pointsmap_renderer/VoxelGridMap.h"

#ifdef _OPENMP
#endif
#include <omp.h>

#define DEFAULT_VOXEL_SIZE 10.0F

enum frustumPointIndex {
    FrontBottomLeft = 0,
    FrontTopLeft = 1,
    FrontTopRight = 2,
    FrontBottomRight = 3,
    BackBottomLeft = 4,
    BackTopLeft = 5,
    BackTopRight = 6,
    BackBottomRight = 7,
    Origin = 8
};

enum frustumSurfaceIndex {
    Left = 0,
    Top = 1,
    Right = 2,
    Bottom = 3,
    Front = 4,
    Back = 5
};

enum frustumVectorIndex {
    Origin2BackBottomLeft = 0,
    Origin2BackTopLeft = 1,
    Origin2BackTopRight = 2,
    Origin2BackBottomRight = 3,
    FrontBottomRight2FrontBottomLeft = 4,
    FrontBottomRight2FrontTopRight = 5,
    BackTopLeft2BackBottomLeft = 6,
    BackTopLeft2BackTopRight = 7
};

enum Axis {
    X = 0,
    Y = 1,
    Z = 2
};

enum K {
    Fx = 0,
    Cx = 2,
    Fy = 4,
    Cy = 5
};

template<typename T>
struct XY {
    T x;
    T y;
};

template<typename T>
struct XYZ {
    T x;
    T y;
    T z;
};

struct Voxel {
    XYZ<float_t> min;
    XYZ<float_t> max;
    pcl::PointCloud<pcl::PointXYZ> points;
};

struct Range {
    float_t min;
    float_t max;
};

template<typename T>
struct LRTB {T l; T r; T t; T b; };

struct CameraParam {
    float_t Fx;
    float_t Fy;
    float_t Cx;
    float_t Cy;
    size_t height;
    size_t width;
};

template <class VoxelGridMapPtrT>
class VoxelGridMap
{
public:
    VoxelGridMap();
    VoxelGridMap(const VoxelGridMapPtrT &vgm);
    VoxelGridMap(const std::string &path, const std::string &frame_id, const float_t voxel_size = DEFAULT_VOXEL_SIZE);
    VoxelGridMap(const std::vector<std::string> &paths, const std::string &frame_id, const float_t voxel_size = DEFAULT_VOXEL_SIZE);
    VoxelGridMap(const sensor_msgs::PointCloud2ConstPtr pointsmap, const float_t voxel_size = DEFAULT_VOXEL_SIZE);

    void set_pointsmap(const std::string &path, const std::string &frame_id, const float_t voxel_size = DEFAULT_VOXEL_SIZE);
    void set_pointsmap(const std::vector<std::string> &paths, const std::string &frame_id, const float_t voxel_size = DEFAULT_VOXEL_SIZE);
    void set_pointsmap(const sensor_msgs::PointCloud2ConstPtr pointsmap, const float_t voxel_size = DEFAULT_VOXEL_SIZE);
    bool is_empty();

    template <typename CameraInfoPtrT>
    void create_depthmap(
        const CameraInfoPtrT &camerainfo,
        const geometry_msgs::TransformStamped &tf,
        cv::Mat &depthmap,
        const Range &depth_range = {.min = 0.0F, .max = INFINITY}
    );

    VoxelGridMapPtrT get_vgm();

private:
    VoxelGridMapPtrT _voxel_grid_map;

    void _invert_transform(const Eigen::Vector3f &in_tr, const Eigen::Quaternionf &in_q, Eigen::Vector3f &out_tr, Eigen::Quaternionf &out_q);
    void _transform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ> &src,
        pcl::PointCloud<pcl::PointXYZ> &dst,
        const Eigen::Vector3f &translation,
        const Eigen::Quaternionf &quaternion
    );
    void _depth_filter(const pcl::PointCloud<pcl::PointXYZ> &src, pcl::PointCloud<pcl::PointXYZ> &dst, const Range &depth_range);
    void _set_voxel_grid(const pcl::PointCloud<pcl::PointXYZ> &pointsmap, const std::string &frame_id, const float_t voxel_size);
    void _create_frustum_points(const Range &depth_range, const LRTB<float_t> &tan, pcl::PointCloud<pcl::PointXYZ> &dst);
    bool _voxel_frontside(const pointsmap_renderer::VoxelPoints &voxel, const Eigen::Vector3f &normal, const Eigen::Vector3f &point_on_plane);
    void _voxels_include_frustum(const Eigen::Vector3f &translation, const Eigen::Quaternionf &quaternion, const Range &depth_range, const LRTB<float_t> &tan, std::vector<XYZ<size_t> > &dst_voxel_indexs);
    void _voxels_include_frustum(const pcl::PointCloud<pcl::PointXYZ> &frustum_points, const Range &depth_range, std::vector<XYZ<size_t> > &dst_voxel_indexs);
    void _create_depthmap(
        const std::vector<XYZ<size_t> > &voxel_indexs,
        const Eigen::Vector3f &translation,
        const Eigen::Quaternionf &quaternion,
        const CameraParam &camera_param,
        const XY<float_t> &shape_f,
        const LRTB<float_t> &tan,
        const Range &depth_range,
        cv::Mat &depth
    );
};
