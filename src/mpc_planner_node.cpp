#include <test_autodock/mpc_planner_ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_planner_node");
    ros::NodeHandle nh;

    // Create an instance of the MPCPlannerROS class
    mpc_ros::MPCPlannerROS planner;

    // Spin the node to process callbacks
    ros::spin();

    return 0;
}