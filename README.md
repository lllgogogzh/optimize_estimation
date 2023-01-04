# optimize_estimation

```shell
catkin_make
roslaunch optimize_estimation test.launch
rosbag xxxxxxxxxxxxxxxxxxxxxxxxx

```



#### new ekf 框架

```shell
catkin_make
rosrun optimize_estimation imu_ekf_node
rosbag xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```



#### new version1 : add ekf

```c++
//TODO: 
//TODO : calc F and Q'
//TODO : calc x_pred (x_pred should be normlized)

//下面预测代码需要修改(我有一些公式不太明白) 里面last_tw 还有 输入噪声协方差COV 没有定义。
double dt = t_now_-t_last_;
Eigen::Matrix4d Fq = Eigen::Matrix4d::Identity();
Eigen::MatrixXd Gq(4, 3);
double qw = state_q_(0), qx = state_q_(1), qy = state_q_(2), qz = state_q_(3);
Gq << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;
Gq = Gq * dt / 2;
x_pred = Fq * state_q_ + Gq * last_tw.segment(1, 3);
x_pred.normalize();

P_pred = Fq*P_last_*Fq.transpose()+Gq*COV*Gq.transpose();
```

