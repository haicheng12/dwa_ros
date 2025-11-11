#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <tf2/LinearMath/Quaternion.h> // 四元数转欧拉角
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <visualization_msgs/Marker.h>

// 机器人参数配置
struct RobotParams
{
    double max_v = 0.5;            // 最大线速度 (m/s)
    double min_v = 0.0;            // 最小线速度 (m/s，支持倒车)
    double max_omega = 2.0;        // 最大角速度 (rad/s)
    double min_omega = -2.0;       // 最小角速度 (rad/s)
    double max_acc_v = 0.2;        // 最大线加速度 (m/s²)
    double max_acc_omega = 1.0;    // 最大角加速度 (rad/s²)
    double wheel_base = 0.2;       // 轮间距 (m)
    double dt = 0.1;               // 控制周期 (s)
    double predict_time = 1.0;     // 预测时域 (s)
    double safe_dist = 0.3;        // 安全距离 (m)
    double goal_dist_thresh = 0.2; // 到达目标的距离阈值 (m)
};

// 位姿结构体 (x, y坐标，航向角theta)
struct Pose
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// 速度结构体 (线速度v，角速度omega)
struct Velocity
{
    double v = 0.0;
    double omega = 0.0;
};

// 障碍物结构体 (x, y坐标)
struct Obstacle
{
    double x;
    double y;
};

// 定义离散点结构体（x, y, z坐标）
struct Point
{
    double x, y, z;
    Point(double x_, double y_, double z_ = 0.0) : x(x_), y(y_), z(z_) {}
};

// 定义轨迹（由多个点组成）
struct Trajectory
{
    std::vector<Point> points; // 轨迹点集
    std::string name;          // 轨迹名称（用于标注）
    double r, g, b;            // 轨迹颜色
    double line_width;         // 线宽
};

class DWA
{
public:
    DWA();
    virtual ~DWA();

    void run();

private:
    // 核心入口：输入当前状态、目标、障碍物，输出最优速度
    Velocity solve(const Pose &current_pose, const Velocity &current_vel,
                   const Pose &goal, const std::vector<Obstacle> &obstacles);

    // 1. 生成动态窗口（可行速度范围）
    std::vector<Velocity> generateDynamicWindow(const Velocity &current_vel);
    // 2. 轨迹预测（基于差速轮运动学模型）
    std::vector<Pose> predictTrajectory(const Pose &current_pose, const Velocity &vel);
    // 3. 避障判断（轨迹是否碰撞）
    bool isCollision(const std::vector<Pose> &trajectory, const std::vector<Obstacle> &obstacles);
    // 4. 评价函数（多维度打分，权重可调）
    double evaluateTrajectory(const std::vector<Pose> &trajectory, const Pose &goal,
                              const std::vector<Obstacle> &obstacles);

    // 里程计回调：更新当前位姿与速度
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    // 激光雷达回调：提取障碍物（转换为机器人坐标系下的坐标）
    // void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    // 目标点回调：更新目标位姿
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    Pose current_pose_;                                          // 当前位姿
    Velocity current_vel_;                                       // 当前速度
    Pose goal_;                                                  // 目标位姿
    std::vector<Obstacle> obstacles_ = {{0.0, 0.0}, {0.0, 0.0}}; // 障碍物列表

    RobotParams params_; // 机器人参数

    bool has_goal_ = false; // 是否收到目标点
    bool has_scan_ = true;  // 是否收到激光数据

protected:
    ros::NodeHandle nh_; // ros节点

    ros::Subscriber odom_sub_; // 订阅里程计
    ros::Subscriber scan_sub_; // 订阅激光雷达
    ros::Subscriber goal_sub_; // 订阅目标点

    ros::Publisher cmd_pub_;    // 发布速度指令
    ros::Publisher marker_pub_; // Marker发布器
};
