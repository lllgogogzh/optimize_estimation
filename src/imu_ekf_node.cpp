#include "optimize_estimation/imu_ekf.h"

using namespace std;

int main(int argc , char **argv)
{
    ros::init(argc,argv,"imu_ekf_node");
    ros::NodeHandle nh;
    IMUEKF my_imu_ekf(nh);
    ros::spin();
}
