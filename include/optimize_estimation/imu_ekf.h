#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>

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

        IMUEKF(ros::NodeHandle &nh);

    public:
    //for ROS
        ros::NodeHandle nh_; 
        ros::Subscriber odom_sub_;
        ros::Subscriber imu_sub_;
        //ros::Subscriber t_a_sub = nh.subscribe<std_msgs::Float64>("/airsim_node/drone_1/t_a",1,TaCallbackFunc);
        //ros::Subscriber cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/airsim_node/drone_1/angle_rate_throttle_enu",1,CmdCallbackFunc);
        ros::Subscriber t_a_sub_;
        ros::Subscriber cmd_sub_;

        ros::Publisher only_imu_pose_pub_;
        ros::Publisher only_process_pose_pub_;
        ros::Publisher ekf_pose_pub_;

    //Callback function
        void TaCallbackFunc(const std_msgs::Float64 msg)
        {
            have_ta_ = true;
            T_a_ = msg.data;
        }

        void CmdCallbackFunc(const mavros_msgs::AttitudeTarget msg);

        void odomCallbackFunc(const nav_msgs::Odometry msg);

        void IMUCallbackFunc(const sensor_msgs::Imu msg);

    public:
    //for initial value
        Eigen::Vector3d init_p_;
        Eigen::Vector3d init_v_;
        Eigen::Vector4d init_q_;
        Eigen::Vector3d init_w_;
        bool have_init_odom_;

        enum SystemState
        {
            INITIAL=0,
            NORMAL=1,
            ERROR=2
        };

        int system_state_;

    public:
    //for process value
        double T_a_;
        bool have_ta_;
        double t_last_;
        Eigen::Vector3d acc_last_;
        Eigen::Vector3d gyro_last_;
        Eigen::Vector4d last_tw_;
        Eigen::Vector4d curr_tw_;

        bool have_imu_;
        Eigen::Vector3d acc_meas_;
        Eigen::Vector3d gyro_meas_;
        Eigen::Matrix3d acc_meas_covirance_; // this is not be initilized !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Eigen::Matrix3d gyro_meas_covirance_;
        //double t_imu_;
    
    //state
        //Eigen::Quaterniond Qwb_;
        typedef Eigen::Matrix<double,6,1> Vector6d;

        Eigen::Vector4d state_q_;
        Eigen::Vector3d state_p_;
        Eigen::Vector3d state_v_;
        Eigen::VectorXd state_pv_;

    public:
    //for EKF
        void EKF_Pose_Estimation();

    //we can have some value in here
        Eigen::Matrix<double,4,3> H_;
        void JacobianMeasurement();



    public:
    //other functions
        void Initialize();
};