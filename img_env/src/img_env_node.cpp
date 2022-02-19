#include <ros/ros.h>
#include "img_env.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "img_env");
    ImgEnv img_env;
    ros::spin();
    return 0;
}
