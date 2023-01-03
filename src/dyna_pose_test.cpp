#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Vector3d init_p(0.0,0.0,0.0);
Eigen::Vector3d init_v(0.0,0.0,0.0);
Eigen::Vector4d init_q(1.0,0.0,0.0,0.0);
Eigen::Vector3d init_w(0.0,0.0,0.0);

Eigen::VectorXd state_pv = Eigen::VectorXd::Zero(6);
Eigen::Vector4d state_q(1.0,0.0,0.0,0.0);
Eigen::Vector4d curr_tw = Eigen::Vector4d::Zero();
Eigen::Vector4d last_tw = Eigen::Vector4d::Zero();

Eigen::Vector3d velocity(0.0,0.0,0.0);
Eigen::Vector3d pose(0.0,0.0,0.0);

Eigen::Vector3d velocity_last(0.0,0.0,0.0);
Eigen::Vector3d pose_last(0.0,0.0,0.0);

Eigen::Vector3d acc_last(0.0,0.0,9.83);
Eigen::Vector3d g(0.0,0.0,-9.83);

Eigen::Quaterniond Qwb(1.0,0.0,0.0,0.0);// wxyz
Eigen::Vector3d gyro_last(0.0,0.0,0.0);

bool have_ta = false, have_init_odom = false, first_input = true;
double t_last = 0.0, T_a;

class IMU_TEST
{
    public:

};


void IMUCallbackFunc(sensor_msgs::ImuConstPtr msgptr)
{
    Eigen::Vector3d gyro_now(msgptr->angular_velocity.x,msgptr->angular_velocity.y,msgptr->angular_velocity.z);
    Eigen::Vector3d gyro_meas = 0.5*(gyro_now+gyro_last);
    gyro_last = gyro_now;

    double t_now = msgptr->header.stamp.toSec();
    if(t_last == 0.0)
    {
        t_last = msgptr->header.stamp.toSec();
    }
    double dt = t_now - t_last;

    Eigen::Quaterniond delta_q;
    delta_q.w()=1.0;
    delta_q.x()=gyro_meas.x()*dt*0.5;
    delta_q.y()=gyro_meas.y()*dt*0.5;
    delta_q.z()=gyro_meas.z()*dt*0.5;

    double ax_m=msgptr->linear_acceleration.x;
    double ay_m=msgptr->linear_acceleration.y;
    double az_m=msgptr->linear_acceleration.z;
    Eigen::Vector3d acc_meas(ax_m,ay_m,az_m);

    Eigen::Matrix3d R_now = Qwb.normalized().toRotationMatrix();
    Eigen::Vector3d acc_w_now = R_now*acc_meas+g;

    Eigen::Vector3d acc = 0.5*(acc_w_now+acc_last);
    acc_last = acc_w_now;
    //Eigen::Vector3d acc = 0.5*(R_now*(acc_last+g)+R_last*(acc_meas+g));

    std::cout<<velocity(0)<<std::endl;
    Qwb = Qwb.normalized()*delta_q.normalized();
    velocity = velocity + acc* dt;
    pose = pose + velocity * dt + 0.5* acc *dt*dt;
    t_last = t_now;

    //pub

}

void odomCallbackFunc(nav_msgs::Odometry::ConstPtr msg){
    if(have_init_odom) return;
    have_init_odom = true;
    init_p(0) = msg->pose.pose.position.x;
    init_p(1) = msg->pose.pose.position.y;
    init_p(2) = msg->pose.pose.position.z;

    init_v(0) = msg->twist.twist.linear.x;
    init_v(1) = msg->twist.twist.linear.y;
    init_v(2) = msg->twist.twist.linear.z;

    init_q(0) = msg->pose.pose.orientation.w;
    init_q(1) = msg->pose.pose.orientation.x;
    init_q(2) = msg->pose.pose.orientation.y;
    init_q(3) = msg->pose.pose.orientation.z;

    init_w(0) = msg->twist.twist.angular.x;
    init_w(1) = msg->twist.twist.angular.y;
    init_w(2) = msg->twist.twist.angular.z;

    state_pv.segment(0, 3) = init_p;
    state_pv.segment(3, 3) = init_v;
    state_q = init_q;
}

void TaCallbackFunc(std_msgs::Float64::ConstPtr msg){
    have_ta = true;
    T_a = msg->data;
}

void CmdCallbackFunc(mavros_msgs::AttitudeTarget::ConstPtr msg){
    if((!have_ta) || (!have_init_odom)) return;
    if(first_input){
        t_last = msg->header.stamp.toSec();
        first_input = false;
        last_tw(0) = msg->thrust * T_a;
        last_tw(1) = msg->body_rate.x;
        last_tw(2) = msg->body_rate.y;
        last_tw(3) = msg->body_rate.z;
    }
    double t_now = msg->header.stamp.toSec();
    double dt = t_now - t_last;
    t_last = t_now;
    curr_tw(0) = msg->thrust * T_a;
    curr_tw(1) = msg->body_rate.x;
    curr_tw(2) = msg->body_rate.y;
    curr_tw(3) = msg->body_rate.z;

    // xpv,k+1 = Fp * xpv,k + Gp,k * up,k
    Eigen::MatrixXd Fp = Eigen::MatrixXd::Identity(6, 6);
    Fp.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
    Eigen::MatrixXd Gp = Eigen::MatrixXd::Zero(6, 2);
    Eigen::Quaterniond q(state_q(0),state_q(1),state_q(2),state_q(3));
    Gp.block(3, 0, 3, 1) = q.normalized().toRotationMatrix().block(0, 2, 3, 1);
    Gp(5, 1) = -1;
    Gp = Gp * dt;
    Eigen::Vector2d u_pv(last_tw(0), 9.81);
    state_pv = Fp * state_pv + Gp * u_pv;

    // xq,k+1 = Fq * xq,k + Gq,k * uq,k
    Eigen::Matrix4d Fq = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Gq(4, 3);
    double qw = state_q(0), qx = state_q(1), qy = state_q(2), qz = state_q(3);
    Gq << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;
    Gq = Gq * dt / 2;
    state_q = Fq * state_q + Gq * last_tw.segment(1, 3);
    state_q.normalize();

    last_tw = curr_tw; 
    
    // have_ta = true;
    // T_a = msg->data;
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_test");
    ros::NodeHandle nh;

    // ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu_enu",1,IMUCallbackFunc);
    ros::Subscriber t_a_sub = nh.subscribe<std_msgs::Float64>("/airsim_node/drone_1/t_a",1,TaCallbackFunc);
    ros::Subscriber cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/airsim_node/drone_1/angle_rate_throttle_enu",1,CmdCallbackFunc);
    ros::Subscriber ground_truth_sub = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_enu",1,odomCallbackFunc);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_test",1);

    //ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
        //pub
        geometry_msgs::PoseStamped posepub;
        posepub.pose.position.x=state_pv(0);
        posepub.pose.position.y=state_pv(1);
        posepub.pose.position.z=state_pv(2);
        posepub.pose.orientation.w = state_q(0);
        posepub.pose.orientation.x = state_q(1);
        posepub.pose.orientation.y = state_q(2);
        posepub.pose.orientation.z = state_q(3);
        posepub.header.frame_id = "odom";
        posepub.header.stamp = ros::Time::now();

        pose_pub.publish(posepub);
    }
    return 0;    
}