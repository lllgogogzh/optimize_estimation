#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "optimize_estimation/matplotlibcpp.h"

namespace plt = matplotlibcpp;

class IMUEKF
{
    public:
    //contrustion function
        IMUEKF()
        {
            ROS_ERROR("we cant init a IMUEKF class like this!!");
        };
        ~IMUEKF()
        {

        };

        IMUEKF(ros::NodeHandle &_nh);

    //for ROS
        ros::NodeHandle nh_; 
        ros::Subscriber odom_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber t_a_sub_;
        ros::Subscriber cmd_sub_;

        ros::Publisher only_imu_pose_pub_;
        ros::Publisher only_process_pose_pub_;
        ros::Publisher ekf_pose_pub_, p_error_pub_, q_error_pub_;

        bool is_pub_pose_, have_plot_;
        void PublishEKFPose();

    //Callback function
        void TaCallbackFunc(const std_msgs::Float64 msg)
        {
            have_ta_ = true;
            T_a_ = msg.data;
        }

        void CmdCallbackFunc(const mavros_msgs::AttitudeTarget msg);

        void odomCallbackFunc(const nav_msgs::Odometry msg);

        void IMUCallbackFunc(const sensor_msgs::Imu msg);

        Eigen::Matrix3d cross_mat(Eigen::Vector3d vec){
            Eigen::Matrix3d mat;
            mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
            return mat;
        }

    //for initial value
        Eigen::Vector3d init_p_;
        Eigen::Vector3d init_v_;
        Eigen::Vector4d init_q_;
        Eigen::Vector3d init_w_;
        bool have_init_odom_;
        Eigen::Matrix4d Pq_hat_, Pq_est_;
        Eigen::MatrixXd Kq_;
        std::vector<double> p_errors_;
        std::vector<double> q_errors_;
        std::vector<double> time_;
        double start_time_;

        enum SystemState
        {
            INITIAL=0,
            NORMAL=1,
            ERROR=2
        };

        int system_state_;

    //for process value
        double T_a_;
        bool have_ta_;
        double t_last_;
        double t_now_;
        Eigen::Vector3d acc_last_;
        Eigen::Vector3d gyro_last_;
        Eigen::Vector4d last_tw_;
        Eigen::Vector4d curr_tw_;

        bool have_imu_;
        Eigen::Vector3d acc_meas_;
        Eigen::Vector3d gyro_meas_;
        Eigen::Matrix3d acc_meas_covirance_; // this is not be initilized !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Eigen::Matrix3d gyro_meas_covirance_;
        Eigen::Vector3d gw_;

        void UpdateStatePV();
        void UpdateStatePV_dyna();
        //double t_imu_;
    
    //state
        //Eigen::Quaterniond Qwb_;
        typedef Eigen::Matrix<double,6,1> Vector6d;

        Eigen::Vector4d state_q_;
        Eigen::Vector3d state_p_;
        Eigen::Vector3d state_v_;
        Eigen::VectorXd state_pv_;

        Eigen::Matrix4d P_last_;

    //for EKF
        void EKF_Pose_Estimation();
        void EKF_Pose_Estimation2();
        

    //we can have some value in here
        Eigen::Matrix<double,4,3> H_;
        void JacobianMeasurement();

    //other functions
        void Initialize();
        void record_error();
};