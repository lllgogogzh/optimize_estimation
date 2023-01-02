#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "nav_msgs/Path.h"

/* 参考ROS wiki
 * http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
 * */

int count=0;

Eigen::Quaterniond Qwb(1.0,0.0,0.0,0.0);//Eigen::Quaterniond的顺序w, x, y z,

Eigen::Vector3d gw(0,0,-9.81);    // ENU frame

Eigen::Vector3d Vw=Eigen::Vector3d::Zero();    //
Eigen::Vector3d Pwb(0.0,0.0,0.0); 
ros::Publisher m_midPath_pub;
void PublishPath(ros::Publisher& puber, std::vector<Eigen::Vector3d> position, std::vector<Eigen::Quaterniond> rotation)
{
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    // pose.header.frame_id = "/world";
    pose.header.frame_id = "/odom";

    for(int i = 0; i < position.size();i++)
    {
        Eigen::Quaterniond Q_from_R = rotation[i];
        Eigen::Vector3d traj_node = position[i];
        pose.pose.position.x = traj_node(0);
        pose.pose.position.y = traj_node(1);
        pose.pose.position.z = traj_node(2);
        pose.pose.orientation.x = Q_from_R.x();
        pose.pose.orientation.y = Q_from_R.y();
        pose.pose.orientation.z = Q_from_R.z();
        pose.pose.orientation.w = Q_from_R.w();
        path_msg.poses.push_back(pose);
    }

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "/odom";
    // path_msg.header.frame_id = "/world";
    puber.publish(path_msg);
}


void ImuCallback(const sensor_msgs::ImuConstPtr& imu_data) {
    static tf::TransformBroadcaster br;//广播器
    tf::Transform transform;

    static std::vector<Eigen::Vector3d> midPosition;
    static std::vector<Eigen::Quaterniond> midRotation; 
    
    static double last_time;
    ++count;
    if(count<=50){
    // std::cout<<imu_data->header.seq<<std::endl;
    last_time=imu_data->header.stamp.toSec();
    return;
    }
    
    double now_time=imu_data->header.stamp.toSec();
    double dt=now_time-last_time;
    // std::cout<<now_time - last_time<<std::endl;
    last_time=now_time;

    // //pre_integration_euler 欧拉积分
    // Eigen::Quaterniond dq;
    // Eigen::Vector3d dtheta_half ={imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z};
    // // std::cout<<"imu_data->angular_velocity: "<<imu_data->angular_velocity<<std::endl;
    // dtheta_half=dtheta_half*dt/2.0;
    // // std::cout<<"dtheta_half "<<dtheta_half<<std::endl;
    // dq.w() = 1;
    // dq.x() = dtheta_half.x();
    // dq.y() = dtheta_half.y();
    // dq.z() = dtheta_half.z();
    // Eigen::Vector3d imu_acc(imu_data->linear_acceleration.x,imu_data->linear_acceleration.y,imu_data->linear_acceleration.z);
    // Eigen::Vector3d acc_w = Qwb.normalized() * (imu_acc)+gw;
    // // std::cout<<Qwb.w()<<"   "<<Qwb.x()<<"   "<<Qwb.y()<<"   "<<Qwb.z()<<std::endl;
    // Qwb = Qwb * dq;
    // Qwb.normalize();
    // std::cout<<"Vw:\n"<<Vw<<std::endl;
    // Vw = Vw + acc_w * dt;
    // Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

    //pre_integration_mid 中值积分
    static Eigen::Vector3d last_acc = Eigen::Vector3d::Zero() ;
    static Eigen::Vector3d last_gyr = Eigen::Vector3d::Zero() ;
    static bool first_flag = true;

    Eigen::Vector3d imu_acc(imu_data->linear_acceleration.x,imu_data->linear_acceleration.y,imu_data->linear_acceleration.z);
    Eigen::Vector3d imu_gyro(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);
    if(first_flag == true)
    {
        first_flag = false;
        // last_acc = Qwb.normalized() * (Eigen::Vector3d::Zero()) + gw;
        // last_gyr =Eigen::Vector3d::Zero();
        last_acc = Qwb.normalized() * (imu_acc) + gw;
        last_gyr =imu_gyro;
    }
    Eigen::Quaterniond dq;
    Eigen::Vector3d dtheta_half =  (imu_gyro + last_gyr) / 2.0 * dt / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    Eigen::Vector3d acc_k = Qwb.normalized() * (imu_acc) + gw;
    Eigen::Vector3d acc_mid = (acc_k + last_acc) / 2.0;  
    Qwb = Qwb.normalized() * dq.normalized();
    Vw = Vw+acc_mid * dt;
    Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_mid;

    // std::cout<<"count "<<count<<std::endl;
    // std::cout<<"dt: "<<dt<<std::endl;
    // std::cout<<"imu_gyro: \n"<<imu_gyro.x()<<"    "<<imu_gyro.y()<<"   "<<imu_gyro.z()<<std::endl;
    // std::cout<<"last_acc: \n"<<last_acc.x()<<"    "<<last_acc.y()<<"   "<<last_acc.z()<<std::endl;
    // std::cout<<"imu_acc: \n"<<imu_acc.x()<<"    "<<imu_acc.y()<<"   "<<imu_acc.z()<<std::endl;
    // std::cout<<"acc_k: \n"<<acc_k.x()<<"    "<<acc_k.y()<<"   "<<acc_k.z()<<std::endl;
    // std::cout<<"acc_mid: \n"<<acc_mid.x()<<"    "<<acc_mid.y()<<"   "<<acc_mid.z()<<std::endl;
    // std::cout<<"Vw: \n"<<Vw.x()<<"    "<<Vw.y()<<"   "<<Vw.z()<<std::endl<<std::endl;
    last_acc = acc_k;
    last_gyr = imu_gyro;

             // save path
    midPosition.push_back(Pwb);
    midRotation.push_back(Qwb);
    
    if(count % 5 == 0)
    {
        PublishPath(m_midPath_pub, midPosition, midRotation);
        std::cout << " mid-integration pub path : " << count << std::endl;
    }


    transform.setOrigin(tf::Vector3(Pwb.x(), Pwb.y(), Pwb.z()));//设置平移部分
    // transform.setOrigin(tf::Vector3(position_x, position_y, position_z));//设置平移部分

    //从IMU消息包中获取四元数数据
    tf::Quaternion q;
    q.setX(Qwb.x());
    q.setY(Qwb.y());
    q.setZ(Qwb.z());
    q.setW(Qwb.w());
    q.normalized();//归一化

    transform.setRotation(q);//设置旋转部分
    //广播出去
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu_sim"));
}
int main (int argc, char ** argv) {
    ros::init(argc, argv, "imu_data_to_tf");
    
    ros::NodeHandle node;

    m_midPath_pub = node.advertise<nav_msgs::Path>("/imu_path", 10);
	//IMU_data改成自己imu的话题就行了
    ros::Subscriber sub = node.subscribe("/airsim_node/drone_1/imu/imu_enu", 10, &ImuCallback);

    ros::spin();
    return 0;
}


