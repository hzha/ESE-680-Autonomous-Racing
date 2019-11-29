
// This file contains the node definition for RRT
// reference to: https://arxiv.org/pdf/1105.1186.pdf


#include "rrt/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::spin();
    return 0;
}
