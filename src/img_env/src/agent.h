/*
 * Created Date: Tuesday, May 5th 2020, 11:26:42 pm 
 * Author: Guangda Chen
 * 
 * Copyright (c) 2020 DRL_NAV
 */
#ifndef   _agent_
#define   _agent_
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include "grid_map.h"

using namespace cv;
using namespace std;

/*
 Agent类用来表示img_env里的机器人或者障碍物
*/
class Agent
{
public:
    Agent();
    Agent(string ktype);
    Agent(string ktype, double resolution);
    // init circle bbox
    void init_shape_circle(const vector<double> &sizes, vector<Point2d>& bbox);
    // init rectangle bbox
    void init_shape_rectangle(const vector<double> &sizes, vector<Point2d>& bbox);
    // init beep circle bbox
    void init_shape_beep(const vector<double> &sizes, double beep_r);
    virtual void init_shape(string shape, const vector<double> &sizes);

    void init_view_map(double width, double height, double resolution);
    void view2base(const Point2d& xy_view, Point2d& xy_base);
    void base2view(const Point2d& xy_base, Point2d& xy_view);
    void base2world(const Point2d& xy_base, Point2d& xy_world, tf::Transform tf_base_world);
    tf::Transform get_base_world();
    tf::Transform get_view_world();
    void init_pose(const Point3d& pose);
    void set_goal(const Point3d& pose);
    void get_state(vector<double>& state);
    // 得到世界坐标系的agent左下角|右上角的2个点坐标
    void get_corners(double& pax, double& pay, double& pbx, double& pby);

    // 用v w速度驱动机器人运动
    bool cmd(double v, double w,double v_y=0.0);
    // 在grid_map上绘制bbox形状
    int draw(GridMap& grid_map, int value, string target_frame, vector<Point2d>& bbox);
    void draw_rgb(GridMap &grid_map, Vec3b value, string target_frame, vector<Point2d>& bbox);
    // 在grid_map上截取机器人视角图
    bool view(GridMap& grid_map);

    double bresenhamLine(int x1, int y1, int x2, int y2, GridMap& source_map, GridMap &target_map);
    vector<double> hits_;
    vector<double> hit_points_x_;
    vector<double> hit_points_y_;
    vector<double> angular_map_;
    int is_collision_;
    bool is_arrive_;
    string ktype_;
    string shape_; // 形状
    vector<double> sizes_; // 尺寸
    string name_; // 名称
    vector<Point2d> bbox_; // 形状像素点

    vector<Point2d> bbox_beep_; // beep 形状像素点 (外面一圈)

    Point3d robot_pose_; // 当前位姿
    Point3d last_robot_pose_;
    Point3d odom_pose_;
    Point3d target_pose_; // 目标点
    GridMap view_map_; // 视角图
    GridMap global_map_; // 全局静态地图

    Point2d last_vw_;
    int state_dim_;
    double control_hz_;
    double step_hz_;

    // 坐标转换
    tf::Transform tf_view_base_;
    tf::Transform tf_base_view_;
    tf::Transform tf_odom_world_;
    tf::Transform tf_target_world_;
    tf::Transform tf_world_target_;

    // 激光参数
    bool use_laser_;
    int range_total_;
    double view_angle_begin_;
    double view_angle_end_;
    double view_min_dist_;
    double view_max_dist_;
    Point2d sensor_base_;

    // beep
    int beep = 0;
    double max_speed;
    double vx;
    double vy;

};

/*
 PedAgent类用来表示img_env里的行人
*/
class PedAgent: public Agent{
public:
    PedAgent(string ktype);
    PedAgent(string ktype, double resolution);
    void set_params();
    void set_position(Point3d pose);
    void update_bbox();
    void init_shape(string shape, const vector<double> &sizes);
    geometry_msgs::Point _get_cur_goal();
    vector<geometry_msgs::Point> trajectory_;
    int cur_traj_index_;
    bool arrive(geometry_msgs::Point p);
    double remaining_dist_;
    Point3d left_leg_;
    Point3d right_leg_;
    double left_leg_r_, right_leg_r_;
    vector<Point2d> left_leg_bbox_, right_leg_bbox_;
    int state_, last_state_;
    double step_len_;
    bool draw_leg(GridMap& grid_map, int value);
    bool draw_leg_rgb(GridMap &grid_map, Vec3b value);
    tf::Transform get_leg_base(Point3d& leg);
    void leg2base(const Point2d &xy_leg, Point2d &xy_base, tf::Transform tf_leg_base);

    double vx;
    double vy;
};
#endif