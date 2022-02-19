/*
 * Created Date: Tuesday, May 5th 2020, 11:26:42 pm 
 * Author: Guangda Chen
 * 
 * Copyright (c) 2020 DRL_NAV
 */

#include "agent.h"
#include "scenefactory.h"
#include <ros/ros.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <comn_pkg/ResetEnv.h>
#include <comn_pkg/StepEnv.h>
#include <comn_pkg/InitEnv.h>
#include <comn_pkg/Env.h>
#include <comn_pkg/EnvsInfo.h>
#include <comn_pkg/RobotRes.h>
#include <comn_pkg/EpRes.h>
#include <comn_pkg/EndEp.h>
#include <comn_pkg/PedInfo.h>
#include <pedsimros/ped_scene.h>
#include <pedsimros/ped_agent.h>
#include <pedsimros/ped_obstacle.h>
#include <pedsimros/ped_waypoint.h>
#include <ervo_ros/RVO.h>
#include <ervo_ros/ERVOSimulator.h>
#include <comn_pkg/drl_console.h>
#include <comn_pkg/peek_cmd.h>
#include <ervo_ros/RVO.h>
using namespace std;
using namespace cv;

/*
 img_env环境信息类，用于区分多个环境信息
*/
class EnvMap
{
public:
    EnvMap() {}
    // 替代方案是只有一个全局地图，再写一个占位符类，对每个占位符进行区分。

    string name_; // 环境名称
    GridMap static_map_; // 全局静态地图(由图片文件载入) 
    GridMap obs_map_; // 障碍物 + 全局静态地图
    GridMap peds_map_; // 行人 + 障碍物 + 全局静态地图
    GridMap peds_traj_map_;
    GridMap traj_map_; // 机器人轨迹 + 行人 + 障碍物 + 全局静态地图
    GridMap visual_map_; // 其他可视化 + 机器人轨迹 + 行人 + 障碍物 + 全局静态地图
    vector<string> robots_name_; // 环境包含的全部机器人名称
    vector<string> peds_name_; // 环境包含的全部行人名称

    void view_new_ped();
};

/*
 img_env环境类，包含整个仿真器
*/
class ImgEnv
{
public:
    ImgEnv();
    ~ImgEnv();

    ros::NodeHandle private_nh;

    string env_name; // 环境名称
    int env_id;

    int robot_total_;
    int ped_total_;
    int env_total_;
    int relation_ped_robo;
    double view_resolution_;
    double step_hz_;
    double sleep_t_;
    double robot_view_width;
    double robot_view_height;
    int state_dim_;

    comn_pkg::EpRes eps_res_msg;
    EnvMap EnvMap_maps_;

    boost::shared_ptr<Scene> pedscene;

    // vector<EnvInfo> envs_; // 多环境信息
    vector<Agent> robots_; // 所有机器人
    vector<PedAgent> peds_; // 所有行人

    map<string, double> pedscene_double_param;
    map<string, vector<double>> pedscene_vecdouble_param;

    void view_ped();//obs_map->ped_map tmp no muti-thread
    void view_robot();
    void view_agent();

    void get_states(vector<comn_pkg::AgentState>& robot_states);

    // 使用ros与python训练程序交互
    cv_bridge::CvImage img_bridge;

    ros::Publisher episode_res_pub_;

    void _init(comn_pkg::Env &msg_env);
    bool _end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res);
    bool _reset(comn_pkg::ResetEnv::Request &req,comn_pkg::ResetEnv::Response &res);
    bool _step(comn_pkg::StepEnv::Request &req,comn_pkg::StepEnv::Response &res);

    void _init_env_map(comn_pkg::Env &msg_env);
    void _init_param_from_py(comn_pkg::InitEnv::Request &req);
    void _init_ped(vector<comn_pkg::Agent> &msg_peds, vector<PedAgent> &peds_);
    void _init_robot(vector<comn_pkg::Agent> &msg_robots, vector<Agent> &robots_);
    void _init_colors();

    // 可视化
    bool is_show_gui_;
    vector<Vec3b> colors_;
    bool is_draw_step_;
    int step_draw_;
    int step_;
    int window_height_;
    int show_image_height_;
    int show_robot_num_;
    void show_gui();

    // 激光
    bool use_laser_;
    int range_total_;
    double view_angle_begin_;
    double view_angle_end_;
    double view_min_dist_;
    double view_max_dist_;

//    void oncmd(const char *cmd);
    bool pub_record_;

    // 喇叭影响范围
    double beep_r;
    // 喇叭影响概率
    double ped_ca_p;
    // 行人最大速度
    vector<double> ped_max_speed_;

};


class EnvService{
public:
    EnvService();
    ~EnvService(){};
    ros::NodeHandle nh_;

    ros::ServiceServer reset_env_srv;
    ros::ServiceServer step_srv;
    ros::ServiceServer init_env_srv;
    ros::ServiceServer ep_end_srv;


    bool reset_env(comn_pkg::ResetEnv::Request &req,comn_pkg::ResetEnv::Response &res);
    bool step_env(comn_pkg::StepEnv::Request &req,comn_pkg::StepEnv::Response &res);
    bool init_env(comn_pkg::InitEnv::Request &req,comn_pkg::InitEnv::Response &res);
    bool end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res);

    vector<ImgEnv> envs_;
    ImgEnv ImgEnv_env;


};