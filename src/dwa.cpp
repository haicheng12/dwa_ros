#include "dwa/dwa.h"

DWA::DWA()
{
    // 初始化ROS订阅与发布
    odom_sub_ = nh_.subscribe("/odom", 1, &DWA::odomCallback, this);
    // scan_sub_ = nh_.subscribe("/scan", 1, &DWA::scanCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWA::goalCallback, this);
    path_sub_ = nh_.subscribe("/coverage_path", 1, &DWA::pathCallback, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sample_trajectory", 10);
}

DWA::~DWA()
{
}

// 里程计回调：更新当前位姿与速度
void DWA::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // 转换四元数为欧拉角（roll, pitch, yaw）
    tf2::Quaternion quat;
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);
    quat.setW(w);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw); // 单位：弧度（rad）

    // 更新位姿（x, y, theta）
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.theta = yaw;

    // 更新速度（v, omega）
    current_vel_.v = msg->twist.twist.linear.x;
    current_vel_.omega = msg->twist.twist.angular.z;

    current_position_.pose = msg->pose.pose;

    // std::cout << "当前状态 = " << current_pose_.x << " " << current_pose_.y << " " << current_pose_.theta << " " << current_vel_.v << " " << current_vel_.omega << std::endl;
}

// 目标点回调：更新目标位姿
void DWA::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;

    // 转换四元数为欧拉角（roll, pitch, yaw）
    tf2::Quaternion quat;
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);
    quat.setW(w);

    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw); // 单位：弧度（rad）

    // 目标位姿
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    // 目标朝向
    goal_.theta = yaw;

    has_goal_ = true;

    std::cout << "收到目标点: " << goal_.x << " " << goal_.y << " " << goal_.theta << std::endl;
}

// 路径回调：接收全局路径
void DWA::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    static bool flag = true;
    if (flag)
    {
        flag = false;

        nav_msgs::Path path = *msg;

        // 遍历存储到容器
        for (int i = 0; i < path.poses.size(); ++i)
        {
            Pose temp_pose;
            temp_pose.x = path.poses[i].pose.position.x;
            temp_pose.y = path.poses[i].pose.position.y;

            coverage_path_.push_back(temp_pose);
        }
        // std::cout << "成功接收到全局路径" << std::endl;

        has_path_ = true;
    }
}

// 核心入口：输入当前状态、目标、障碍物，输出最优速度
Velocity DWA::solve(const Pose &current_pose, const Velocity &current_vel,
                    const Pose &goal, const std::vector<Obstacle> &obstacles)
{
    // 1. 生成动态窗口
    std::vector<Velocity> window = generateDynamicWindow(current_vel);

    // 2. 遍历窗口，筛选最优轨迹
    double best_score = -1.0;
    Velocity best_vel = {0.0, 0.0};

    // std::cout << "window size = " << window.size() << std::endl;//45条轨迹
    for (int i = 0; i < window.size(); ++i) // 45条轨迹
    {
        Velocity vel; // 动态窗口采样的速度
        vel.v = window[i].v;
        vel.omega = window[i].omega;

        // // 避障判断（碰撞轨迹直接跳过）
        // if (isCollision(traj, obstacles))
        // {
        //     continue;
        // }

        // 预测轨迹
        std::vector<Pose> traj = predictTrajectory(current_pose, vel); // 11个点

        pubTrajectory(traj, i); // 发布轨迹

        // 评价轨迹得分
        double score = evaluateTrajectory(traj, goal, obstacles);
        // std::cout << "评价轨迹得分 = " << score << std::endl;

        // 更新最优速度
        if (score > best_score)
        {
            best_score = score;
            best_vel = vel;

            // std::cout << "最优速度 = " << best_vel.v << " " << best_vel.omega << std::endl;
        }
    }

    return best_vel;
}

// 1. 生成动态窗口（可行速度范围）
std::vector<Velocity> DWA::generateDynamicWindow(const Velocity &current_vel)
{
    std::vector<Velocity> window;

    // 计算速度约束范围（硬件限制 + 加速度限制）
    double v_min = std::max(params_.min_v, current_vel.v - params_.max_acc_v * params_.dt);
    double v_max = std::min(params_.max_v, current_vel.v + params_.max_acc_v * params_.dt);
    double omega_min = std::max(params_.min_omega, current_vel.omega - params_.max_acc_omega * params_.dt);
    double omega_max = std::min(params_.max_omega, current_vel.omega + params_.max_acc_omega * params_.dt);

    // 速度空间采样（可调整采样密度，这里v分5档，omega分10档）
    double v_step = (v_max - v_min) / 4;
    double omega_step = (omega_max - omega_min) / 9;

    for (double v = v_min; v <= v_max; v += v_step)
    {
        for (double omega = omega_min; omega <= omega_max; omega += omega_step)
        {
            window.push_back({v, omega});
        }
    }

    return window;
}

// 2. 轨迹预测（基于差速轮运动学模型）
std::vector<Pose> DWA::predictTrajectory(const Pose &current_pose, const Velocity &vel)
{
    std::vector<Pose> trajectory;
    Pose pose = current_pose;
    trajectory.push_back(pose);

    // 按预测时域分步计算轨迹
    int steps = params_.predict_time / params_.dt;
    for (int i = 0; i < steps; ++i)
    {
        // 离散运动学更新
        pose.x += vel.v * cos(pose.theta) * params_.dt;
        pose.y += vel.v * sin(pose.theta) * params_.dt;
        pose.theta += vel.omega * params_.dt;
        // 角度归一化到[-π, π]
        pose.theta = atan2(sin(pose.theta), cos(pose.theta));
        trajectory.push_back(pose);
    }

    return trajectory;
}

// 3. 避障判断（轨迹是否碰撞）
bool DWA::isCollision(const std::vector<Pose> &trajectory, const std::vector<Obstacle> &obstacles)
{
    for (const auto &pose : trajectory)
    {
        for (const auto &obs : obstacles)
        {
            // 计算轨迹点与障碍物的距离
            double dist = sqrt(pow(pose.x - obs.x, 2) + pow(pose.y - obs.y, 2));
            if (dist < params_.safe_dist)
            {
                return true; // 碰撞
            }
        }
    }
    return false; // 无碰撞
}

// 4. 评价函数（多维度打分，权重可调）
double DWA::evaluateTrajectory(const std::vector<Pose> &trajectory, const Pose &goal,
                               const std::vector<Obstacle> &obstacles)
{
    // 权重配置（可通过实验调优）
    double w_heading = 1.0; // 朝向权重
    double w_dist = 1.0;    // 避障距离权重
    double w_vel = 1.0;     // 速度权重

    // 4.1 朝向评价（轨迹终点与目标的航向偏差）
    Pose end_pose = trajectory.back();
    double goal_theta = atan2(goal.y - end_pose.y, goal.x - end_pose.x);
    double heading_err = fabs(end_pose.theta - goal_theta);
    // std::cout << "heading_err = " << heading_err << std::endl; // 0.1 多条轨迹的朝向偏差
    double heading_cost = 1.0 - heading_err / M_PI; // 得分0~1
    // std::cout << "朝向评价 = " << heading_cost << std::endl;

    // 4.2 避障距离评价（轨迹与障碍物的最小距离）
    // double min_dist = 1e9;
    // for (const auto &pose : trajectory)
    // {
    //     for (const auto &obs : obstacles)
    //     {
    //         double dist = sqrt(pow(pose.x - obs.x, 2) + pow(pose.y - obs.y, 2));
    //         min_dist = std::min(min_dist, dist);
    //     }
    // }
    // double dist_cost = std::min(min_dist / params_.safe_dist, 1.0); // 得分0~1
    // double dist = sqrt(pow(end_pose.x - goal.x, 2) + pow(end_pose.y - goal.y, 2));
    // double dist_cost = dist;
    // std::cout << "距离评价 = " << dist_cost << std::endl;

    // 4.3 速度评价（线速度接近最大速度的程度）
    double vel = (sqrt(pow(trajectory.back().x - trajectory.front().x, 2) + pow(trajectory.back().y - trajectory.front().y, 2))) / params_.predict_time; // 提取线速度
    // std::cout << "vel = " << vel << std::endl;
    // double vel_cost = vel / params_.max_v; // 得分0~1
    double vel_cost = std::max(0.0, vel) / params_.max_v; // 正向速度优先
    // std::cout << "速度评价 = " << vel_cost << std::endl;
    // vel_cost_data_ = vel_cost;

    // 总得分（加权求和）
    double all_cost = heading_cost * w_heading + vel_cost * w_vel;
    // std::cout << "总评价 = " << all_cost << std::endl;

    return all_cost;
}

void DWA::pubTrajectory(const std::vector<Pose> &trajectory, const int &num) // 发布轨迹
{
    // 发布显示其中一条轨迹
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "line_strip";
    marker.id = num;
    marker.type = visualization_msgs::Marker::LINE_STRIP; // 类型：折线
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    // 颜色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // 填充线的点数据（按顺序连接）
    for (int i = 0; i < trajectory.size(); ++i)
    {
        geometry_msgs::Point pt;
        pt.x = trajectory[i].x;
        pt.y = trajectory[i].y;
        pt.z = 0.0;
        marker.points.push_back(pt);
    }
    marker.lifetime = ros::Duration(0);
    marker_pub_.publish(marker); // 预测轨迹发布显示
}

// 启动机器人
void DWA::runRobot(double v, double omega)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;
    cmd_pub_.publish(cmd);
}

// 停止机器人
void DWA::stopRobot()
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_.publish(cmd);
}

// PID相关
// 设置目标值
void DWA::setTarget(double target)
{
    target_ = target;

    integral_ = 0; // 重置积分项（避免累积误差）
}

// 设置PID参数
void DWA::setParams(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// 设置输出限制
void DWA::setOutputLimit(double limit)
{
    output_limit_ = limit;
}

// 获取误差
double DWA::getError()
{
    return error_;
}

// 计算PID输出（输入当前值，返回控制量）
double DWA::compute(double current, float dt)
{
    if (dt <= 0)
        return 0; // 避免除以零

    // 计算当前误差
    last_error_ = error_;
    error_ = target_ - current;

    // 积分项（带抗积分饱和）
    integral_ += error_ * dt;
    // 积分限幅：当输出达到上限时，停止积分累积
    if (fabs(output_limit_) > 1e-6)
    {
        double integral_max = output_limit_ / ki_;
        integral_ = std::max(-integral_max, std::min(integral_, integral_max));
    }

    // 微分项（用差分近似）
    derivative_ = (error_ - last_error_) / dt;

    // 计算输出并限幅
    double output = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    return std::max(-output_limit_, std::min(output, output_limit_));
}

// 主循环
void DWA::run()
{
    // 串级PID控制器
    DWA outer_pid_; // 外环：角度环（输出目标角速度）
    DWA inner_pid_; // 内环：角速度环（输出角加速度/轮速差）

    // 初始化PID参数（需根据实际机器人调优）
    // 外环：角度环（kp=2.0, ki=0.1, kd=0.5，输出限幅±2.0rad/s）
    outer_pid_.setParams(2.0, 0.1, 0.5);
    outer_pid_.setOutputLimit(2.0);

    // 内环：角速度环（kp=1.5, ki=0.3, kd=0.1，输出限幅±0.5m/s²（轮速差））
    inner_pid_.setParams(1.5, 0.3, 0.1);
    inner_pid_.setOutputLimit(0.5);

    int index = 0;

    ros::Time current_time, last_time; // 当前时间，上一刻时间
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    float dt = 0.0; // 采样时间

    double target_yaw = 0.0; // 朝向角度

    ros::Rate rate(10); // 控制频率
    while (ros::ok())
    {
        if (has_goal_) // 接收到目标点
        {
            current_time = ros::Time::now();

            dt = (current_time - last_time).toSec(); // 时间间隔
                                                     // std::cout << "dt = " << dt << std::endl;

            // 纯跟踪算法计算跟踪曲率
            double cal_rpy = tf::getYaw(current_position_.pose.orientation);
            // std::cout << "cal_rpy " << cal_rpy << std::endl;
            double alpha = atan2(goal_.y - current_position_.pose.position.y, goal_.x - current_position_.pose.position.x) - cal_rpy;
            // std::cout << "alpha " << alpha << std::endl;
            double curvature_k = 2 * sin(alpha) / params_.wheel_base; // 跟踪曲率 k = 2 * sin(a) / Ld
            // std::cout << "curvature_k " << curvature_k << std::endl;
            // double k_cost = atan2(sin(curvature_k), cos(curvature_k)); // 归一化得分0~1
            // std::cout << "k_cost " << k_cost << std::endl;

            // 根据曲率对准目标点
            double vel_theta = 0.2 * curvature_k;
            runRobot(0.0, vel_theta); // 启动机器人
            if (curvature_k < 0.05)   // 瞄准好之后发送速度
            {
                // 求解DWA得到最优速度
                Velocity optimal_vel = solve(current_pose_, current_vel_, goal_, obstacles_);

                // 执行速度指令，更新当前状态
                current_pose_.x += optimal_vel.v * cos(current_pose_.theta) * params_.dt;
                current_pose_.y += optimal_vel.v * sin(current_pose_.theta) * params_.dt;
                current_pose_.theta += optimal_vel.omega * params_.dt;
                current_vel_ = optimal_vel;

                runRobot(optimal_vel.v, optimal_vel.omega); // 启动机器人

                // 检查是否到达目标（距离阈值0.2m）
                double dist_to_goal = sqrt(pow(current_pose_.x - goal_.x, 2) + pow(current_pose_.y - goal_.y, 2));
                // std::cout << "dist_to_goal = " << dist_to_goal << std::endl;
                if (dist_to_goal < params_.goal_dist_thresh) // 到目标位置的阈值
                {
                    std::cout << "到达目标点！" << std::endl;
                    stopRobot(); // 停止机器人

                    // // 1.设置目标旋转角度
                    // outer_pid_.setTarget(goal_.theta);
                    // std::cout << "开始旋转到目标角度: " << goal_.theta << std::endl;

                    // // 这里开始旋转对准目标点
                    // // 2. 外环PID（角度环）：计算目标角速度
                    // double target_omega = outer_pid_.compute(current_pose_.theta, dt);
                    // // 3. 内环PID（角速度环）：计算轮速差（转化为角速度控制量）
                    // double omega_cmd = inner_pid_.compute(current_vel_.omega, dt);
                    // // 4. 检查是否到达目标角度
                    // double angle_error = outer_pid_.getError(); // 获取外环误差

                    // if (fabs(angle_error) < params_.angle_tolerance) // 到目标角度阀值
                    // {
                    //     stopRobot();       // 停止机器人
                    //     has_goal_ = false; // 旋转完成，重置目标标志

                    //     std::cout << "旋转完成！当前角度:" << current_pose_.theta << "误差:" << angle_error << std::endl;
                    // }

                    // // 5. 发布速度指令（仅旋转，线速度为0）
                    // runRobot(0.0, omega_cmd);

                    // // 调试信息
                    // std::cout << "角度误差:" << angle_error << ",目标角速度: " << target_omega << ",当前角速度:" << current_vel_.omega << ",输出指令:" << omega_cmd << std::endl;
                }
            }

            last_time = current_time;
        }

        // if (has_path_) // 接收到路径点
        // {
        //     current_time = ros::Time::now();
        //     dt = (current_time - last_time).toSec(); // 时间间隔
        //     // std::cout << "dt = " << dt << std::endl;

        //     // std::cout << "size = " << coverage_path_.size() << std::endl; // 11
        //     // for (int i = 0; i < coverage_path_.size(); ++i)
        //     // {
        //     //     double goal_x = coverage_path_[i].x;
        //     //     double goal_y = coverage_path_[i].y;
        //     //     // 打印点
        //     //     std::cout << "位置 = " << goal_x << " " << goal_y << std::endl;
        //     // }

        //     double goal_x = coverage_path_[index].x;
        //     double goal_y = coverage_path_[index].y;
        //     // 打印点
        //     // std::cout << "位置 = " << goal_x << " " << goal_y << std::endl;

        //     // 纯跟踪算法计算跟踪曲率
        //     double cal_rpy = tf::getYaw(current_position_.pose.orientation);
        //     // std::cout << "cal_rpy " << cal_rpy << std::endl;
        //     double alpha = atan2(goal_y - current_position_.pose.position.y, goal_x - current_position_.pose.position.x) - cal_rpy;
        //     // std::cout << "alpha " << alpha << std::endl;
        //     double curvature_k = 2 * sin(alpha) / params_.wheel_base; // 跟踪曲率 k = 2 * sin(a) / Ld
        //     std::cout << "curvature_k " << curvature_k << std::endl;

        //     // 根据曲率对准目标点
        //     double vel_theta = 0.2 * curvature_k;
        //     runRobot(0.0, vel_theta); // 启动机器人
        //     if (curvature_k < 0.1)    // 瞄准好之后发送速度
        //     {
        //         Pose current_goal;
        //         current_goal.x = goal_x;
        //         current_goal.y = goal_y;
        //         current_goal.theta = 0.0;

        //         // 求解DWA得到最优速度
        //         Velocity optimal_vel = solve(current_pose_, current_vel_, current_goal, obstacles_);

        //         // 执行速度指令，更新当前状态
        //         current_pose_.x += optimal_vel.v * cos(current_pose_.theta) * params_.dt;
        //         current_pose_.y += optimal_vel.v * sin(current_pose_.theta) * params_.dt;
        //         current_pose_.theta += optimal_vel.omega * params_.dt;
        //         current_vel_ = optimal_vel;

        //         runRobot(optimal_vel.v, optimal_vel.omega); // 启动机器人

        //         std::cout << "index = " << index << std::endl;

        //         // 检查是否到达目标（距离阈值0.2m）
        //         double dist_to_goal = sqrt(pow(current_pose_.x - current_goal.x, 2) + pow(current_pose_.y - current_goal.y, 2));
        //         // std::cout << "dist_to_goal = " << dist_to_goal << std::endl;
        //         if (dist_to_goal < 0.3) // 到目标位置的阈值
        //         {
        //             std::cout << "去下一个目标点！" << std::endl;
        //             ++index;

        //             if (index == coverage_path_.size())
        //             {
        //                 std::cout << "到达终点！" << std::endl;

        //                 index = 0; // 重复清扫
        //                 // stopRobot(); // 停止机器人
        //             }
        //         }
        //     }

        //     last_time = current_time;
        // }

        rate.sleep();
        ros::spinOnce();
    }
}