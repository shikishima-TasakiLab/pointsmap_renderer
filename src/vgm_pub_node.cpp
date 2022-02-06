#include "vgm_pub.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxelgridmap_publisher");
    VGM_Pub vgm_pub;
    ros::spin();
    return EXIT_SUCCESS;
}
