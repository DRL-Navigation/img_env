/*
 * Created Date: Tuesday, May 5th 2020, 11:26:42 pm 
 * Author: Guangda Chen
 * 
 * Copyright (c) 2020 DRL_NAV
 */

#include "agent.h"
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

using namespace std;
using namespace cv;

/*
 img_env环境信息类，用于区分多个环境信息
*/
class EnvInfo
{
public:
    EnvInfo() {}
    string name_; // 环境名称
    GridMap static_map_; // 全局静态地图(由图片文件载入) 
    GridMap obs_map_; // 障碍物 + 全局静态地图
    GridMap peds_map_; // 行人 + 障碍物 + 全局静态地图
    GridMap peds_traj_map_;
    GridMap traj_map_; // 机器人轨迹 + 行人 + 障碍物 + 全局静态地图
    GridMap visual_map_; // 其他可视化 + 机器人轨迹 + 行人 + 障碍物 + 全局静态地图
    vector<string> robots_name_; // 环境包含的全部机器人名称
    vector<string> peds_name_; // 环境包含的全部行人名称
};

/*
 img_env环境类，包含整个仿真器
*/
class ImgEnv : public DRLConsole
{
public:
    ImgEnv();
    ~ImgEnv();
    int robot_total_;
    int env_total_;
    int relation_ped_robo;
    double view_resolution_;
    double step_hz_;
    double sleep_t_;
    vector<comn_pkg::EpRes> eps_res_msg;

    vector<EnvInfo> envs_; // 多环境信息
    vector<Agent> robots_; // 所有机器人
    vector<PedAgent> peds_; // 所有行人

    map<string, int> envs_index_; // 环境名称和envs_序号对应关系
    map<string, int> robots_index_; // 机器人名称和robots_序号对应关系
    map<string, string> robots_envs_; // 机器人名称和环境名称对应关系
    map<string, string> peds_envs_; // 行人名称和环境名称对应关系
    map<string, int> peds_index_; // 行人名称和peds_序号对应关系
    map<string, int> rvo_index_;

    // Pedsim的仿真环境
    vector<Ped::Tscene*> pedscenes_;
    vector<Ped::Tagent *> robots_sim_;
    vector<Ped::Tobstacle *> obs_sim_;
    vector<Ped::Tagent *> peds_sim_;

    // RVO
    vector<RVO::RVOSimulator *> RVOscenes_;

    // ERVO
    vector<RVO::ERVOSimulator *> ERVOscenes_;

    // 截取机器人视角图的多线程
    int max_thread_;
    int thread_total_;
    std::vector<boost::thread*> thread_vector_;
    std::vector<int> thread_status_;
    void view_thread(int begin_index, int end_index, int thread_num);
    void view_ped();//obs_map->ped_map tmp no muti-thread
    void get_states(vector<comn_pkg::AgentState>& robot_states);

    // 使用ros与python训练程序交互
    ros::NodeHandle nh_;
    cv_bridge::CvImage img_bridge;
    ros::ServiceServer reset_env_srv;
    ros::Publisher episode_res_pub_;
    ros::ServiceServer ep_end_srv;
    bool end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res);
    bool reset_env(comn_pkg::ResetEnv::Request &req,comn_pkg::ResetEnv::Response &res);
    ros::ServiceServer step_srv;
    bool step_env(comn_pkg::StepEnv::Request &req,comn_pkg::StepEnv::Response &res);
    ros::ServiceServer init_env_srv;
    bool init_env(comn_pkg::InitEnv::Request &req,comn_pkg::InitEnv::Response &res);

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

    void oncmd(const char *cmd);
    bool pub_record_;
};
