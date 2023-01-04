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

    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("imu_ekf_pose",1);
    is_pub_pose_ = true;

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
    gw_<<0.0,0.0,-9.80;

    double sigma2 = 0.05;
    acc_meas_covirance_(0,0) = sigma2;
    acc_meas_covirance_(1,1) = sigma2;
    acc_meas_covirance_(2,2) = sigma2;

    //Kalman
    P_last_  = Eigen::Matrix4d::Identity()*0.01;
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
        t_now_ = msg.header.stamp.toSec();
        //Do EKF
        ROS_INFO("Lets do some ekf!!"); 
        EKF_Pose_Estimation();
        if(is_pub_pose_)
        {

        }
        t_last_ = t_now_; 
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
    gyro_last_ = init_w_;
}

void IMUEKF::EKF_Pose_Estimation()
{
    //TODO EKF

    //Predicit
    Eigen::Matrix4d P_pred;
    Eigen::Vector4d x_pred;
    //TODO : calc F and Q'
    //TODO : calc x_pred (x_pred should be normlized)
    double dt = t_now_-t_last_;
    Eigen::Matrix4d Fq = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Gq(4, 3);
    double qw = state_q_(0), qx = state_q_(1), qy = state_q_(2), qz = state_q_(3);
    Gq << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;
    Gq = Gq * dt / 2;
    x_pred = Fq * state_q_ + Gq * last_tw.segment(1, 3);
    x_pred.normalize();

    P_pred = Fq*P_last_*Fq.transpose()+Gq*COV*Gq.transpose();

    //Kalman Update
    //calc H :
    double q0,q1,q2,q3;// wxyz
    Eigen::Matrix<double,3,4> H;
    q0 = state_q_(0);
    q1 = state_q_(1);
    q2 = state_q_(2);
    q3 = state_q_(3);
    H << -q2 , q3 , -q0 , q1,
               q1 , q0 , q3 , q2,
               q0 , -q1 , -q2 , q3;
    
    //Kalman zengyi K
    Eigen::Matrix<double,4,3> K;
    Eigen::Matrix<double,3,3> T;

    T = H*P_pred*H.transpose()+acc_meas_covirance_;
    K=P_pred*H.transpose()*T.inverse();

    //update
    Eigen::Matrix4d P_est;
    Eigen::Vector4d x_est;
    Eigen::Vector3d y,gk;
    //y : the real measurement
    y(0) = acc_meas_(0);
    y(1) = acc_meas_(1);
    y(2) = acc_meas_(2);
    //gk : in the measurement function with x_pred
    gk(0) = 2*x_pred(1)*x_pred(3) - 2*x_pred(0)*x_pred(2);
    gk(1) = 2*x_pred(2)*x_pred(3) - 2*x_pred(0)*x_pred(1);
    gk(2) = x_pred(0)*x_pred(0) - x_pred(1)*x_pred(1) - x_pred(2)*x_pred(2) - x_pred(3)*x_pred(3);
    P_est=(Eigen::Matrix4d::Identity() - K*H)*P_pred;
    x_est=x_pred + K*(y - gk);

    //update the optimize estimation
    state_q_ = x_est;
    P_last_ = P_est;

    UpdateStatePV();

}

void IMUEKF::UpdateStatePV()
{
    //update pv by imu
    double dt = t_now_ - t_last_;

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d gyro_now_ = 0.5*(gyro_last_+gyro_meas_);
    delta_q.w()=1.0;
    delta_q.x()=gyro_now_.x()*dt*0.5;
    delta_q.y()=gyro_now_.y()*dt*0.5;
    delta_q.z()=gyro_now_.z()*dt*0.5;

    Eigen::Quaterniond Qwb(state_q_(0),state_q_(1),state_q_(2),state_q_(3));

    Eigen::Matrix3d R_now = Qwb.normalized().toRotationMatrix();
    Eigen::Vector3d acc_w_now = R_now*acc_meas_+gw_;

    Eigen::Vector3d acc = 0.5*(acc_w_now+acc_last_);
    acc_last_ = acc_w_now;
    //Eigen::Vector3d acc = 0.5*(R_now*(acc_last+g)+R_last*(acc_meas+g));

    Qwb = Qwb.normalized()*delta_q.normalized();
    state_v_ = state_v_+ acc* dt;
    state_p_ = state_p_ +state_v_ * dt + 0.5* acc *dt*dt;

    state_pv_.segment(0,3) = state_p_;
    state_pv_.segment(3,3) = state_v_;
}

void IMUEKF::PublishEKFPose()
{
    geometry_msgs::PoseStamped pubpose;
    pubpose.header.frame_id = "odom";
    pubpose.header.stamp = ros::Time::now();

    pubpose.pose.orientation.w = state_q_(0);
    pubpose.pose.orientation.x = state_q_(1);
    pubpose.pose.orientation.y = state_q_(2);
    pubpose.pose.orientation.z = state_q_(3);

    pubpose.pose.position.x = state_pv_(0);
    pubpose.pose.position.y = state_pv_(1);
    pubpose.pose.position.z = state_pv_(2);

    ekf_pose_pub_.publish(pubpose);
}

