#include "optimize_estimation/imu_ekf.h"

IMUEKF::IMUEKF(ros::NodeHandle &nh)
{
    nh_=nh;
    ROS_INFO("Start initialize!!");
    Initialize();
}

void IMUEKF::Initialize()
{
    system_state_ = SystemState::INITIAL;

    //ROS
    //in callback function , we cannot use ConstPtr be the function value.
    cmd_sub_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("/airsim_node/drone_1/angle_rate_throttle_enu",1,&IMUEKF::CmdCallbackFunc,this);
    t_a_sub_ = nh_.subscribe<std_msgs::Float64>("/airsim_node/drone_1/t_a",1,&IMUEKF::TaCallbackFunc,this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_enu",1,&IMUEKF::odomCallbackFunc,this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu_enu",1,&IMUEKF::IMUCallbackFunc,this);

    //init value
    state_pv_ = Eigen::VectorXd::Zero(6);
    state_q_=Eigen::Vector4d::Zero();
    state_q_(0)=1; // wxyz

    acc_last_ = Eigen::Vector3d::Zero();
    gyro_last_ = Eigen::Vector3d::Zero();
    curr_tw_ = Eigen::Vector4d::Zero();
    last_tw_ = Eigen::Vector4d::Zero();
    T_a_=0.0;
    t_last_=0.0;
    have_ta_ = false;

    init_p_ = Eigen::Vector3d::Zero();
    init_v_ = Eigen::Vector3d::Zero();
    init_q_=Eigen::Vector4d::Zero();
    init_w_=Eigen::Vector3d::Zero();
    have_init_odom_ = false;

    acc_meas_ = Eigen::Vector3d::Zero();
    gyro_meas_ = Eigen::Vector3d::Zero();
    have_imu_ = false;
}

void IMUEKF::CmdCallbackFunc(const mavros_msgs::AttitudeTarget msg)
{
    if(system_state_ == SystemState::INITIAL)
    {
        if(have_init_odom_==false||have_ta_==false||have_imu_==false)
        {
            //no init odom, cannot initilaze
            return;
        }

        // initilize something i dont no
        system_state_ = SystemState::NORMAL;
        t_last_ = msg.header.stamp.toSec();
        last_tw_(0) = msg.thrust * T_a_;
        last_tw_(1) = msg.body_rate.x;
        last_tw_(2) = msg.body_rate.y;
        last_tw_(3) = msg.body_rate.z;

        // initilize imu
        acc_last_ = acc_meas_;
        gyro_last_ = gyro_meas_;
    }
    else if(system_state_ == SystemState::NORMAL)
    {
        //Do EKF
        ROS_INFO("Lets do some ekf!!"); 
        EKF_Pose_Estimation();
    }
    else if(system_state_ == SystemState::ERROR)
    {
        ROS_ERROR("ERROR!!!!!!!!!!!!!!!!");
    }
}

void IMUEKF::IMUCallbackFunc(const sensor_msgs::Imu msg)
{
    have_imu_ = true;
    acc_meas_(0) = msg.linear_acceleration.x;
    acc_meas_(1) = msg.linear_acceleration.y;
    acc_meas_(2) = msg.linear_acceleration.z;

    gyro_meas_(0) = msg.angular_velocity.x;
    gyro_meas_(1) = msg.angular_velocity.y;
    gyro_meas_(2) = msg.angular_velocity.z;
}

void IMUEKF::odomCallbackFunc(const nav_msgs::Odometry msg)
{
    if(have_init_odom_) 
    {
        return;
    }
    have_init_odom_ = true;
    init_p_(0) = msg.pose.pose.position.x;
    init_p_(1) = msg.pose.pose.position.y;
    init_p_(2) = msg.pose.pose.position.z;

    init_v_(0) = msg.twist.twist.linear.x;
    init_v_(1) = msg.twist.twist.linear.y;
    init_v_(2) = msg.twist.twist.linear.z;

    init_q_(0) = msg.pose.pose.orientation.w;
    init_q_(1) = msg.pose.pose.orientation.x;
    init_q_(2) = msg.pose.pose.orientation.y;
    init_q_(3) = msg.pose.pose.orientation.z;

    init_w_(0) = msg.twist.twist.angular.x;
    init_w_(1) = msg.twist.twist.angular.y;
    init_w_(2) = msg.twist.twist.angular.z;

    state_pv_.segment(0, 3) = init_p_;
    state_pv_.segment(3, 3) = init_v_;
    state_q_ = init_q_;
}

void IMUEKF::EKF_Pose_Estimation()
{
    //TODO EKF

    //Predicit


    //Kalman Update
    //calc Jacobian
    //K= ???????????



    //update
    //x=???
    //P=???
}

