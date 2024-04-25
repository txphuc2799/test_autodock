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
 */

#include <test_autodock/mpc_planner_ros.h>

using namespace std;
using namespace Eigen;

namespace mpc_ros{

MPCPlannerROS::MPCPlannerROS():
    as_(_nh, "start_mpc", boost::bind(&MPCPlannerROS::executeCb, this, _1), false)
{  
    //we're all set up now so we can start the action server
    as_.start();

    initialize();
}
MPCPlannerROS::~MPCPlannerROS()
{
}

void MPCPlannerROS::initialize(){

    ros::NodeHandle private_nh("~/");
    
    // Publishers and Subscribers:
    cmd_vel_pub_     = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _sub_odom        = _nh.subscribe("odom", 1, &MPCPlannerROS::odomCB, this);
    _sub_global_plan = _nh.subscribe("/plan", 1, &MPCPlannerROS::pathCB, this);

    // init robot state
    x_odom_ = 0.0;
    y_odom_ = 0.0;
    theta_odom_ = 0.0;
    vx_odom_ = 0.0;
    vy_odom_ = 0.0;
    vw_odom_ = 0.0;

    // init mpc params
    private_nh.param<double>("control_frequency", _control_frequency, 10.0);
    private_nh.param<double>("mpc_steps", _mpc_steps, 20.0);
    private_nh.param<double>("ref_cte", _ref_cte, 0.0);
    private_nh.param<double>("ref_etheta",_ref_etheta, 0.0);
    private_nh.param<double>("ref_vel",_ref_vel, 0.6);
    private_nh.param<double>("w_cte",_w_cte, 2000.0);
    private_nh.param<double>("w_etheta",_w_etheta, 500.0);
    private_nh.param<double>("w_vel",_w_vel, 1000.0);
    private_nh.param<double>("w_angvel",_w_angvel, 0.0);
    private_nh.param<double>("w_accel",_w_accel, 10.0);
    private_nh.param<double>("w_angvel_d",_w_angvel_d, 10.0);
    private_nh.param<double>("w_accel_d",_w_accel_d, 10.0);
    private_nh.param<double>("max_angvel",_max_angvel, 0.6);
    private_nh.param<double>("max_throttle",_max_throttle, 1.0);
    private_nh.param<double>("bound_value",_bound_value, 1.0e3);
    private_nh.param<double>("path_length", _path_length, 5.0);
    private_nh.param<double>("max_linear_speed", _max_linear_speed, 0.7);
    private_nh.param<double>("min_linear_speed", _min_linear_speed, -0.3);
    private_nh.param<double>("xy_tolerance", _xy_tolerance, 0.05);
    private_nh.param<double>("yaw_tolerance", _yaw_tolerance, 0.05);
    _dt = double(1.0/_control_frequency);

    setParam();

    //Init variables
    _throttle = 0.0; 
    _w = 0.0;
    _speed = 0.0;

    ROS_INFO("Initialized MPC Controller.");
}

void MPCPlannerROS::setParam(){
    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_etheta;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_ANGVEL"]  = _w_angvel;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["ANGVEL"]   = _max_angvel;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params);      
}

bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    getLocalPlan(transformed_plan);

    bool isOk = mpcComputeVelocityCommands(cmd_vel, transformed_plan);
    if (!isOk)
        ROS_WARN("[MPCROS] failed to produce path.");
    return isOk;
}

// Timer: Control Loop (closed loop nonlinear MPC)
bool MPCPlannerROS::mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                                const std::vector<geometry_msgs::PoseStamped> transformed_plan)
{
    //compute what trajectory to drive along
    geometry_msgs::Twist drive_cmds;

    // call with updated footprint
    Trajectory path = findBestPath(drive_cmds, transformed_plan);

    //pass along drive commands
    cmd_vel = drive_cmds;

    //if we cannot move... tell someone
    if(path.cost_ < 0) {
        ROS_WARN("[MPCROS] cannot find best Trajectory..");
        return false;
    }

    ROS_DEBUG_NAMED("MPCROS", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    return true;
}

Trajectory MPCPlannerROS::findBestPath(geometry_msgs::Twist& drive_velocities,
                                        const std::vector<geometry_msgs::PoseStamped> transformed_plan){

    Trajectory result_traj_;
    result_traj_.cost_ = 1;

    /*
    *
    *  MPC Control Loop
    * 
    */
    // Update system states: X=[x, y, theta, v]
    const double px = x_odom_; //pose: odom frame
    const double py = y_odom_;
    double theta = theta_odom_;
    const double v = hypot(vx_odom_,vy_odom_); //twist: body fixed frame
    // Update system inputs: U=[w, throttle]
    const double w = _w; // steering -> w
    //const double steering = _steering;  // radian
    const double throttle = _throttle; // accel: >0; brake: <0
    const double dt = _dt;

    //Update path waypoints (conversion to odom frame)
    nav_msgs::Path odom_path = nav_msgs::Path();
    double total_length = 0.0;
    int sampling = _downSampling;

    //find waypoints distance
    if(_waypointsDist <=0.0)
    {        
        double dx = transformed_plan[1].pose.position.x - transformed_plan[0].pose.position.x;
        double dy = transformed_plan[1].pose.position.y - transformed_plan[0].pose.position.y;
        _waypointsDist = sqrt(dx*dx + dy*dy);
        _downSampling = 2;
    }
    if (transformed_plan.size() > _downSampling*2){
        // Cut and downsampling the path
        for(int i =0; i< transformed_plan.size(); i++)
        {
            if(total_length > _path_length)
                break;

            if(sampling == _downSampling)
            {
                odom_path.poses.push_back(transformed_plan[i]);  
                sampling = 0;
            }
            total_length = total_length + _waypointsDist; 
            sampling = sampling + 1;  
        }
    } else{
        odom_path.poses = transformed_plan;
    }
    
    if(odom_path.poses.size() < 1)
    {
        ROS_WARN("[MPCROS] Failed to path generation since small down-sampling path.");
        _waypointsDist = -1;
        result_traj_.cost_ = -1;
        return result_traj_;
    }

    // Waypoints related parameters
    const int N = odom_path.poses.size(); // Number of waypoints
    const double costheta = cos(theta);
    const double sintheta = sin(theta);
    
    // Convert to the vehicle coordinate system
    VectorXd x_veh(N);
    VectorXd y_veh(N);
    for(int i = 0; i < N; i++) 
    {
        const double dx = odom_path.poses[i].pose.position.x - px;
        const double dy = odom_path.poses[i].pose.position.y - py;
        x_veh[i] = dx * costheta + dy * sintheta;
        y_veh[i] = dy * costheta - dx * sintheta;
    }

    // Fit waypoints
    auto coeffs = polyfit(x_veh, y_veh, 3); 
    const double cte  = polyeval(coeffs, 0.0);
    double etheta = atan(coeffs[1]);

    // Global coordinate system about theta
    double gx = 0;
    double gy = 0;
    int N_sample = N * 0.3;
    if (N_sample < 1) N_sample = 1; 
    gx = odom_path.poses[N_sample].pose.position.x - odom_path.poses[0].pose.position.x;
    gy = odom_path.poses[N_sample].pose.position.y - odom_path.poses[0].pose.position.y;
    

    double temp_theta = theta;
    double traj_deg = atan2(gy,gx); // odom frame
    double PI = 3.141592;

    // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
    if(temp_theta <= -PI + traj_deg) 
        temp_theta = temp_theta + 2 * PI;
    
    // Implementation about theta error more precisly
    if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
        etheta = temp_theta - traj_deg;
    else
        etheta = 0;  

    VectorXd state(6);

    if(_delay_mode)
    {   
        // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt), body frame
        const double px_act = v * dt;
        const double py_act = 0;
        const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
        const double v_act = v + throttle * dt; //v = v + a * dt
        
        const double cte_act = cte + v * sin(etheta) * dt;
        const double etheta_act = etheta - theta_act;  
        
        state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
        state << 0, 0, 0, v, cte, etheta;
    }

    // Solve MPC Problem
    vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        
    // MPC result (all described in car frame), output = (acceleration, w)        
    _w = mpc_results[0]; // radian/sec, angular velocity
    _throttle = mpc_results[1]; // acceleration m/sec^2

    _speed = v + _throttle * dt;  // speed
    if (_speed >= _max_linear_speed)
        _speed = _max_linear_speed;
    if(_speed <= _min_linear_speed)
        _speed = _min_linear_speed;

    if(result_traj_.cost_ >= 0){
        drive_velocities.linear.x = _speed;
        drive_velocities.angular.z = _w;
    }
    
    return result_traj_;
}

void MPCPlannerROS::runRotationMotion(geometry_msgs::Twist& cmd_vel){
    double _etheta;
    int _index = given_path_.poses.size() - 1;
    _etheta = tf::getYaw(given_path_.poses[_index].pose.orientation) - theta_odom_;
    cmd_vel.angular.z = _etheta + _yaw_tolerance*_etheta/abs(_etheta);
}

// Evaluate a polynomial.
double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

void MPCPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg;
    x_odom_     = _odom.pose.pose.position.x;
    y_odom_     = _odom.pose.pose.position.y;
    theta_odom_ = tf::getYaw(_odom.pose.pose.orientation);
    vx_odom_    = _odom.twist.twist.linear.x;
    vy_odom_    = _odom.twist.twist.linear.y;
    vw_odom_    = _odom.twist.twist.angular.z;
}

void MPCPlannerROS::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    given_path_ = *pathMsg;
}

void MPCPlannerROS::getLocalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
    if (given_path_.poses.empty()) return;

    double rx = _odom.pose.pose.position.x;
    double ry = _odom.pose.pose.position.y;
    
    double max_pow = 10e5;
    int iter_num = 0;
    for (int i = 0; i < given_path_.poses.size(); i++)
    {
        double _dx = rx - given_path_.poses[i].pose.position.x;
        double _dy = ry - given_path_.poses[i].pose.position.y;
        double _pow = _dx*_dx + _dy*_dy;

        if (max_pow > _pow)
        {
            max_pow = _pow;
            iter_num = i;
            continue;
        }
        break;
    }
    for (int j=iter_num; j<given_path_.poses.size(); j++)
    {
        transformed_plan.push_back(given_path_.poses[j]);
    }
}

void MPCPlannerROS::executeCb(const test_autodock::MPCGoalConstPtr &goal)
{
    ros::Rate rate(_control_frequency);

    geometry_msgs::Twist command_vel;
    bool should_rotate = false;

    int last_index = given_path_.poses.size() - 1;

    while (ros::ok())
    {   
        double dx    = given_path_.poses[last_index].pose.position.x - x_odom_;
        double dy    = given_path_.poses[last_index].pose.position.y - y_odom_;
        double theta = tf::getYaw(given_path_.poses[0].pose.orientation) - theta_odom_;
        double distance = hypot(dx, dy);

        ROS_INFO("Remaining distance to goal: %lfm", distance);

        feedback_.remaining_distance = distance;
        as_.publishFeedback(feedback_);

        if ((distance <= _xy_tolerance) && !should_rotate){
            should_rotate = true;
            result_.is_success = true;
            as_.setSucceeded(result_);
            break;
        }
        else if ((distance >= _xy_tolerance) && !should_rotate){
            bool computeDone = computeVelocityCommands(command_vel);
            if (computeDone){
                cmd_vel_pub_.publish(command_vel);
            }
        }
        else if ((abs(theta) > _yaw_tolerance) && should_rotate){
            command_vel.linear.x  = 0.0;
            double sign = theta > 0 ? 1.0 : -1.0;
            command_vel.angular.z = sign*0.05;
            cmd_vel_pub_.publish(command_vel); 
        }
        else{
            ROS_INFO("Reached goal!");
            result_.is_success = true;
            as_.setSucceeded(result_);
            break;
        }

        rate.sleep();
    }
}
}