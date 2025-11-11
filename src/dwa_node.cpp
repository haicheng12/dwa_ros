#include "dwa/dwa.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_ros_test");

    // 初始化DWA算法
    DWA dwa;
    // 主循环
    dwa.run();

    return 0;
}
