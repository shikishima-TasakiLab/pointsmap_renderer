#include "vgm_pub.hpp"

// コンストラクタ
VGM_Pub::VGM_Pub()
{
    // NodeHandleの初期化
    this->_nh.reset(new ros::NodeHandle("~"));

    // voxel_sizeの設定を取得
    this->_nh->getParam("voxel_size", this->_voxel_size);
    ROS_INFO("voxel_size: %f", this->_voxel_size);

    // NodeHandleの初期化
    this->_nh.reset(new ros::NodeHandle("~"));
    // Publisherの初期化
    this->_pub.reset(new ros::Publisher(this->_nh->advertise<pointsmap_renderer::VoxelGridMap>("/voxel_grid_map", 2)));

    // init_mapsの設定の取得
    XmlRpc::XmlRpcValue init_maps;
    this->_nh->getParam("init_maps", init_maps);
    // init_mapsの設定が存在する場合，地図を読み込んでPublish
    if (init_maps.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::string map_frameid = DEFAULT_MAP_FRAMEID;
        this->_nh->getParam("map_frameid", map_frameid);

        std::vector<std::string> map_paths;
        for (size_t i = 0UL; i < init_maps.size(); i++) {
            if (init_maps[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                map_paths.push_back(static_cast<std::string>(init_maps[i]));
            }
        }
        VoxelGridMap<pointsmap_renderer::VoxelGridMapPtr> vgm(map_paths, map_frameid, this->_voxel_size);
        pointsmap_renderer::VoxelGridMapPtr vgm_msg = vgm.get_vgm();
        this->_pub->publish(vgm_msg);
    }

    // Subscriberの初期化
    this->_sub.reset(new ros::Subscriber(this->_nh->subscribe<sensor_msgs::PointCloud2>("/points_map", 2, &VGM_Pub::_callback, this)));
}

// Subscriberのcallback関数
void VGM_Pub::_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 取得した三次元点群地図の読込
    VoxelGridMap<pointsmap_renderer::VoxelGridMapPtr> vgm(msg, this->_voxel_size);
    pointsmap_renderer::VoxelGridMapPtr vgm_msg = vgm.get_vgm();
    // VoxelGridMapのPublish
    this->_pub->publish(vgm_msg);
}
