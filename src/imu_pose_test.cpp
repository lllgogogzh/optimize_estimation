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

Eigen::Vector3d acc_last(0.0,0.0,0.0);
Eigen::Vector3d g(0.0,0.0,-9.83);

double t_last;

class IMU_TEST
{
    public:

};


void IMUCallbackFunc(sensor_msgs::ImuConstPtr msgptr)
{
    double w=msgptr->orientation.w;
    double x=msgptr->orientation.x;
    double y=msgptr->orientation.y;
    double z=msgptr->orientation.z;
    Eigen::Quaterniond q(w,x,y,z);

    double ax_m=msgptr->linear_acceleration.x;
    double ay_m=msgptr->linear_acceleration.y;
    double az_m=msgptr->linear_acceleration.z;
    Eigen::Vector3d acc_meas(ax_m,ay_m,az_m);

    Eigen::Matrix3d R_now = q.toRotationMatrix();
    Eigen::Vector3d acc_w_now = R_now*(acc_meas+g);

    Eigen::Vector3d acc = 0.5*(acc_w_now+acc_last);
    acc_last = acc_w_now;
    //Eigen::Vector3d acc = 0.5*(R_now*(acc_last+g)+R_last*(acc_meas+g));

    double t_now = msgptr->header.stamp.toSec();
    if(t_last == 0.0)
    {
        t_last = msgptr->header.stamp.toSec();
    }
    double dt = t_now - t_last;

    std::cout<<velocity(0)<<std::endl;
    
    pose = pose + velocity * dt + 0.5* acc *dt*dt;
    velocity = velocity + acc* dt;

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
        posepub.pose.position.x=pose(1);
        posepub.pose.position.y=-pose(0);
        posepub.pose.position.z=0.0;
        posepub.header.frame_id = "odom";
        posepub.header.stamp = ros::Time::now();

        pose_pub.publish(posepub);
    }
    return 0;    
}