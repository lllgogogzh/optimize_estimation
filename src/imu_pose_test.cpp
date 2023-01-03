#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Vector3d velocity(0.0,0.0,0.0);
Eigen::Vector3d pose(0.0,0.0,0.0);

Eigen::Vector3d velocity_last(0.0,0.0,0.0);
Eigen::Vector3d pose_last(0.0,0.0,0.0);

Eigen::Vector3d acc_last(0.0,0.0,9.83);
Eigen::Vector3d g(0.0,0.0,-9.83);

Eigen::Quaterniond Qwb(1.0,0.0,0.0,0.0);// wxyz
Eigen::Vector3d gyro_last(0.0,0.0,0.0);

double t_last;

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

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_test");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu_enu",1,IMUCallbackFunc);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_test",1);

    //ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
        //pub
        geometry_msgs::PoseStamped posepub;
        posepub.pose.position.x=pose(0);
        posepub.pose.position.y=pose(1);
        posepub.pose.position.z=pose(2);
        posepub.pose.orientation.w = Qwb.w();
        posepub.pose.orientation.x = Qwb.x();
        posepub.pose.orientation.y = Qwb.y();
        posepub.pose.orientation.z = Qwb.z();
        posepub.header.frame_id = "odom";
        posepub.header.stamp = ros::Time::now();

        pose_pub.publish(posepub);
    }
    return 0;    
}