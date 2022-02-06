#include "pointsmap_renderer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointsmap_renderer");
    Pointsmap_Renderer pr;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return EXIT_SUCCESS;
}
