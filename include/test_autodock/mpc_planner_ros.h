/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 *  Editor : OkDoky(2023)
 */

#ifndef MPC_LOCAL_PLANNER_NODE_ROS_H
#define MPC_LOCAL_PLANNER_NODE_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>
#include <math.h>
#include <fstream>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <test_autodock/mpc_planner.h>
#include <test_autodock/trajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <test_autodock/MPCAction.h>

namespace mpc_ros{

class MPCPlannerROS
{
public:
    MPCPlannerROS();
    ~MPCPlannerROS();

    void initialize();
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                    const std::vector<geometry_msgs::PoseStamped> transformed_plan);
    Trajectory findBestPath(geometry_msgs::Twist& drive_velocities,
                            const std::vector<geometry_msgs::PoseStamped> transformed_plan);
    void runRotationMotion(geometry_msgs::Twist& cmd_vel);
    void setParam(); 
    void getLocalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan);

private:      
    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);

    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    
    void executeCb(const test_autodock::MPCGoalConstPtr &goal);

    /**
     * Use this function if the target is behind the robot, otherwise not use!
    */
    double flipYaw(double yaw);
    
    double polyeval(Eigen::VectorXd coeffs, double x);
    
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

private:
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> mpc_x;
    vector<double> mpc_y;
    vector<double> mpc_theta;

    ros::NodeHandle _nh;
    ros::Subscriber _sub_odom, _sub_global_plan;
    ros::Publisher _pub_downsampled_path, _pub_mpctraj;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cte_pub_;
    ros::ServiceClient _client_set_start;
    tf2_ros::Buffer *tf_;

    actionlib::SimpleActionServer<test_autodock::MPCAction> as_;

    test_autodock::MPCFeedback feedback_;
    test_autodock::MPCResult result_;
    
    nav_msgs::Odometry _odom;
    nav_msgs::Path _odom_path, _mpc_traj, given_path_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::Twist _cmd_vel;

    // init robot pose
    double x_odom_, y_odom_, theta_odom_;
    double vx_odom_, vy_odom_, vw_odom_; // linear x vel, linear y vel, angular yaw vel

    // init goal tolerance
    double _xy_tolerance, _yaw_tolerance;

    double _default_max_linear_speed, _default_max_angular_speed;
    double _safety_speed;

    MPC _mpc;
    map<string, double> _mpc_params;
    double _control_frequency, _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
        _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value, _path_length,
        _max_linear_speed, _min_linear_speed;

    double _dt, _w, _throttle, _speed, _max_speed;
    double _pathLength, _goalRadius, _waypointsDist;
    int _downSampling;
    bool _debug_info, _delay_mode;
    bool initialized_;
    std::string str_state_before;

};
};
#endif /* MPC_LOCAL_PLANNER_NODE_ROS_H */