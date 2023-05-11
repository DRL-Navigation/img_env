#include "img_env.h"

ImgEnv::ImgEnv()
{

    pub_record_ = false;
    private_nh = ros::NodeHandle("~");
    episode_res_pub_ = private_nh.advertise<comn_pkg::EpRes>("episode_res", 100);
    ROS_INFO("NAME:%s", episode_res_pub_.getTopic().c_str());
    _init_colors();

    ROS_INFO("Wait for the python environment to initialize!");
}

//void ImgEnv::oncmd(const char *cmd)
//{
//    float f1, f2, f3, f4, f5, f6, f7, f8, f9, f10;
//    int d1, d2;
//    char str1[50] = "";
//    char str2[50] = "";
//    char str3[50] = "";
//    char str4[50] = "";
//    if (PEEK_CMD(cmd, "show"))
//    {
//        if (is_show_gui_)
//            is_show_gui_ = false;
//        else
//            is_show_gui_ = true;
//    }
//    else if (PEEK_CMD(cmd, "record"))
//    {
//        pub_record_ = true;
//    }
//}
void ImgEnv::_init_colors(){
    colors_.push_back(Vec3b(255, 179, 0)); // vivid_yellow
    colors_.push_back(Vec3b(128, 62, 117)); // strong_purple
    colors_.push_back(Vec3b(255, 104, 0)); // vivid_orange
    colors_.push_back(Vec3b(166, 189, 215)); // very_light_blue
    colors_.push_back(Vec3b(193, 0, 32)); // vivid_red
    colors_.push_back(Vec3b(206, 162, 98)); // grayish_yellow
    colors_.push_back(Vec3b(129, 112, 102)); // medium_gray
    colors_.push_back(Vec3b(0, 125, 52)); // vivid_green
    colors_.push_back(Vec3b(246, 118, 142)); // strong_purplish_pink
    colors_.push_back(Vec3b(0, 83, 138)); // strong_blue
    colors_.push_back(Vec3b(255, 122, 92)); // strong_yellowish_pink
    colors_.push_back(Vec3b(83, 55, 122)); // strong_violet
    colors_.push_back(Vec3b(255, 142, 0)); // vivid_orange_yellow
    colors_.push_back(Vec3b(179, 40, 81)); // strong_purplish_red
    colors_.push_back(Vec3b(244, 200, 0)); // vivid_greenish_yellow
    colors_.push_back(Vec3b(127, 24, 13)); // strong_reddish_brown
    colors_.push_back(Vec3b(147, 170, 0)); // vivid_yellowish_green
    colors_.push_back(Vec3b(89, 51, 21)); // deep_yellowish_brown
    colors_.push_back(Vec3b(241, 58, 19)); // vivid_reddish_orange
    colors_.push_back(Vec3b(35, 44, 22)); // dark_olive_green
}

void ImgEnv::_init_param_from_py(comn_pkg::InitEnv::Request &req){
    robot_view_width = req.view_width;
    robot_view_height = req.view_height;
    view_resolution_ = req.view_resolution;
    state_dim_ = req.state_dim;
    sleep_t_ = req.sleep_t;
    window_height_ = req.window_height;
    show_image_height_ = req.show_image_height;
    is_show_gui_ = req.is_show_gui;
    is_draw_step_ = req.is_draw_step;
    step_draw_ = req.step_draw;
    use_laser_ = req.use_laser;
    range_total_ = req.range_total;
    view_angle_begin_ = req.view_angle_begin;
    view_angle_end_ = req.view_angle_end;
    view_min_dist_ = req.view_min_dist;
    view_max_dist_ = req.view_max_dist;
    show_robot_num_ = int(window_height_ / show_image_height_);

    pedscene_double_param["beep_r"] = beep_r = req.beep_r ;
    pedscene_double_param["ped_ca_p"] = ped_ca_p = req.ped_ca_p;
    pedscene_double_param["relation_ped_robo"] = relation_ped_robo = req.relation_ped_robo;
    pedscene_double_param["step_hz_"] = step_hz_ = req.step_hz;
}

void ImgEnv::_init(comn_pkg::Env &msg_env)
{
    step_ = 0;
    robots_.clear();
    env_id = msg_env.env_id;
    env_name = msg_env.env_name;
    // init pedscene
    SceneFactory scene_f = SceneFactory();// todo add boudaries

    pedscene = scene_f.makeAscene(msg_env.ped_scene_type, pedscene_double_param, pedscene_vecdouble_param);
    pedscene->scene_name_ = msg_env.ped_scene_type;

    _init_env_map(msg_env);
    _init_ped(msg_env.peds, peds_);
    _init_robot(msg_env.robots, robots_);
    

    pedscene->addPed(msg_env.peds.size(), ped_max_speed_);
    pedscene->addRobot(msg_env.robots.size());

}

void ImgEnv::_init_env_map(comn_pkg::Env &msg_env){
    EnvMap_maps_ = EnvMap();

    EnvMap_maps_.static_map_.resolution_ = msg_env.global_resolution;
    EnvMap_maps_.static_map_.read_image(msg_env.map_file, view_resolution_);
    EnvMap_maps_.obs_map_ = EnvMap_maps_.static_map_;
}

void ImgEnv::_init_robot(vector<comn_pkg::Agent> &msg_robots, vector<Agent> &robots_){
    robot_total_ = msg_robots.size();
    for (int j = 0; j < robot_total_; j++)
    {
            Agent robot(msg_robots[j].ktype, step_hz_);
            robot.state_dim_ = state_dim_;
            robot.init_view_map(robot_view_width, robot_view_height, view_resolution_);
            vector<double> rob_size;
            for (int k = 0; k < msg_robots[j].size.size(); k++)
                rob_size.push_back(msg_robots[j].size[k]);
            robot.init_shape(msg_robots[j].shape, rob_size);
            robot.init_shape_beep(rob_size, beep_r);
            robot.use_laser_ = use_laser_;
            robot.range_total_ = range_total_;
            robot.view_angle_begin_ = view_angle_begin_;
            robot.view_angle_end_ = view_angle_end_;
            robot.view_min_dist_ = view_min_dist_;
            robot.view_max_dist_ = view_max_dist_;
            robot.sensor_base_.x = msg_robots[j].sensor_cfg[0];
            robot.sensor_base_.y = msg_robots[j].sensor_cfg[1];
            // speed limiter
            SpeedLimiter speed_limiter_v_ = SpeedLimiter(msg_robots[j].speed_limiter_v);
            SpeedLimiter speed_limiter_w_ = SpeedLimiter(msg_robots[j].speed_limiter_w);
//          speed_limiter_v_.has_acceleration_limits = true;
//          speed_limiter_v_.max_acceleration = 2.0;
//          speed_limiter_v_.min_acceleration = -2.0;
            robot.init_speed_limiter_v(speed_limiter_v_);
            robot.init_speed_limiter_w(speed_limiter_w_);
            robots_.push_back(robot);
    }
}

void ImgEnv::_init_ped(vector<comn_pkg::Agent> &msg_peds, vector<PedAgent> &peds_){
    ped_total_ = msg_peds.size();
    for (int k = 0; k < ped_total_; k++)
    {
            PedAgent ped(msg_peds[k].ktype, step_hz_);
            ped.max_speed = msg_peds[k].max_speed;
            // TODO 下面3行一行搞定。
            vector<double> ped_size;
            for (int m = 0; m < msg_peds[k].size.size(); m++)
                ped_size.push_back(msg_peds[k].size[m]);

            ped.init_shape(msg_peds[k].shape, ped_size);
            peds_.push_back(ped);
            ped_max_speed_.push_back(ped.max_speed);
    }
}

bool ImgEnv::_reset(comn_pkg::ResetEnv::Request &req, comn_pkg::ResetEnv::Response &res){
    private_nh.param("show_gui", is_show_gui_, is_show_gui_);
    step_ = 0 ;
    // reset obs
    pedscene->clearObs();
    EnvMap_maps_.obs_map_ = EnvMap_maps_.static_map_;

    for (int i = 0; i < req.obstacles.size(); i++)
    {
        // 提取python生成的obs信息
        comn_pkg::Agent obs_msg = req.obstacles[i];
        Agent obs("obs", view_resolution_);
        // TODO 下面三行应该有一句话的方法
        vector<double> obs_size;
        for (int j = 0; j < obs_msg.size.size(); j++)
            obs_size.push_back(obs_msg.size[j]);

        obs.init_shape(obs_msg.shape, obs_size);
        tf::Quaternion q(obs_msg.init_pose.orientation.x, obs_msg.init_pose.orientation.y,
                         obs_msg.init_pose.orientation.z, obs_msg.init_pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        Point3d obs_p(obs_msg.init_pose.position.x, obs_msg.init_pose.position.y, yaw);
        obs.init_pose(obs_p);
        // 障碍物占位符为0
        obs.draw(EnvMap_maps_.obs_map_, 0, "world_map", obs.bbox_);
        double pax, pay, pbx, pby;
        obs.get_corners(pax, pay, pbx, pby);

        if(! req.ignore_obstacle)
            pedscene->addObs(pax, pay, pbx, pby);
    }

    EnvMap_maps_.traj_map_ = EnvMap_maps_.obs_map_;

    cvtColor(EnvMap_maps_.traj_map_.map_, EnvMap_maps_.traj_map_.map_, COLOR_GRAY2BGR);

    // TODO reinit pub_record_
    if (pub_record_)
        {
            EnvMap_maps_.peds_traj_map_ = EnvMap_maps_.obs_map_;

            cvtColor(EnvMap_maps_.peds_traj_map_.map_, EnvMap_maps_.peds_traj_map_.map_, COLOR_GRAY2BGR);

        }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1,
                                        EnvMap_maps_.obs_map_.map_);
    img_bridge.toImageMsg(eps_res_msg.obs_map);
        eps_res_msg.env_name = env_name;
        eps_res_msg.resolution = view_resolution_;
        eps_res_msg.step_hz = step_hz_;
        eps_res_msg.robots_res.resize(robot_total_);
        eps_res_msg.peds_res.resize(ped_total_);


    for (int i=0; i<req.peds.size(); i++)
    {
        // TODO env.peds = req.peds;
        eps_res_msg.peds_res[i].info = req.peds[i];
        eps_res_msg.peds_res[i].poses.clear();
        eps_res_msg.peds_res[i].vs.clear();
        eps_res_msg.peds_res[i].v_ys.clear();

        comn_pkg::Agent ped_msg = req.peds[i];
        eps_res_msg.peds_res[i].poses.push_back(ped_msg.init_pose);
        tf::Quaternion q(ped_msg.init_pose.orientation.x, ped_msg.init_pose.orientation.y,
                         ped_msg.init_pose.orientation.z, ped_msg.init_pose.orientation.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // 这里都是世界坐标系
        Point3d ped_p(ped_msg.init_pose.position.x, ped_msg.init_pose.position.y, yaw);
        peds_[i].init_pose(ped_p);

        Point3d ped_goal(ped_msg.goal.x, ped_msg.goal.y, 0);
        peds_[i].set_goal(ped_goal);

        pedscene->setPedPos(i, ped_msg.init_pose.position.x, ped_msg.init_pose.position.y);
        pedscene->setWayPoint(i, ped_msg.trajectory, ped_goal.x, ped_goal.y);
        peds_[i].trajectory_.clear();
        peds_[i].trajectory_ = ped_msg.trajectory;
        peds_[i].trajectory_v_.clear();
        peds_[i].trajectory_v_ = ped_msg.trajectory_v;

        peds_[i].cur_traj_index_ = 0;
    }
    
    for (int i = 0; i < robot_total_; i++)
    {
        // TODO put into init
        string robot_name = req.robots[i].name;
        eps_res_msg.robots_res[i].info = req.robots[i];
//        eps_res_msg.robots_res[i].info.shape = robots_[i].shape_;
//        eps_res_msg.robots_res[i].info.size = robots_[i].sizes_;
        eps_res_msg.robots_res[i].poses.clear();
        eps_res_msg.robots_res[i].vs.clear();
        eps_res_msg.robots_res[i].ws.clear();
        comn_pkg::Agent robot_msg = req.robots[i];
        eps_res_msg.robots_res[i].poses.push_back(robot_msg.init_pose);
        tf::Quaternion q(robot_msg.init_pose.orientation.x, robot_msg.init_pose.orientation.y,
                         robot_msg.init_pose.orientation.z, robot_msg.init_pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        Point3d rob_p(robot_msg.init_pose.position.x, robot_msg.init_pose.position.y, yaw);
        robots_[i].init_pose(rob_p);
        Point3d rob_goal(robot_msg.goal.x, robot_msg.goal.y, 0);
        robots_[i].set_goal(rob_goal);
        robots_[i].is_collision_ = false;
        robots_[i].is_arrive_ = false;

        // change position of robots in pedsim
        double pax, pay, pbx, pby;
        robots_[i].get_corners(pax, pay, pbx, pby);
        if(relation_ped_robo == 1){
            pedscene->setRobotPos(i, pax, pay, pbx, pby, robot_msg.init_pose.position.x, robot_msg.init_pose.position.y,
            0.0, 0.0);
        }
    }
    pedscene->processObs();
    // TODO how to deal with return robot_states in multi spin? // loop
    view_agent();
    get_states(res.robot_states);
    if (is_show_gui_){
        show_gui();
    }

    return true;
}

void ImgEnv::_step_ped(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res){
    if (pedscene->scene_name_ == "dataset"){
        _step_ped_dataset(req, res);
    }
    else{
        // go to rvo or sfm
        _step_ped_normal(req, res);
    }
}

void ImgEnv::_step_ped_normal(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res){
    vector<RVO::Vector2> next_goals;
    for(int i = 0 ;i<peds_.size();i++){
//        if (peds_[i].cur_traj_index_ == -1 || peds_[i].trajectory_.size() == 0 || (peds_[i].arrive(peds_[i].trajectory_[-1])))
//        {
//            peds_[i].cur_traj_index_ = -1;
//            next_goals.push_back(RVO::Vector2(peds_[i].target_pose_.x, peds_[i].target_pose_.y));
//        }
//        else
//        {
            if (peds_[i].arrive(peds_[i].trajectory_[peds_[i].cur_traj_index_]))
                peds_[i].cur_traj_index_ += 1;
            geometry_msgs::Point cur_ped_goal = peds_[i]._get_cur_goal();
            next_goals.push_back(RVO::Vector2(cur_ped_goal.x, cur_ped_goal.y));
//        }
    }
    vector<RVO::Vector2> ps_;
    vector<float> rs_;
        // probability of ped respond to beep
        for (int j = 0; j < req.robots.size(); j++)
        {
            bool is_beep = false;
            robots_[j].beep = 0;
            if (rand() / double(RAND_MAX) < ped_ca_p){
                double beep_radius = req.robots[j].v_y;
                if (beep_radius > 0)
                   {
                        Point3d rpose = robots_[j].robot_pose_;
                        ps_.push_back(RVO::Vector2(rpose.x, rpose.y));
                        rs_.push_back(beep_r);
                        is_beep = true;
                        robots_[j].beep = 1;
                   }
            }
            if (is_beep == false){
                ps_.push_back(RVO::Vector2(0, 0));
                rs_.push_back(0.0);
            }
        }
    pedscene->step(next_goals, ps_, rs_);
    for (int i = 0; i < peds_.size(); i++)
    {
        double vx, vy, yaw;
        geometry_msgs::Pose p_msg;
        pedscene->getNewPosAndVel(p_msg, i, vx, vy);
        Point3d p_robot_pose_(p_msg.position.x, p_msg.position.y, yaw);
        peds_[i].set_position(p_robot_pose_);
        peds_[i].vx = vx;
        peds_[i].vy = vy;
        peds_[i].update_bbox();

        eps_res_msg.peds_res[i].vs.push_back(vx);
        eps_res_msg.peds_res[i].v_ys.push_back(vy);
        eps_res_msg.peds_res[i].poses.push_back(p_msg);
    }
}

void ImgEnv::_step_ped_dataset(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res){
    for (int i = 0; i < peds_.size(); i++)
    {
        int tmp_traj_index;
        if (step_ >= peds_[i].trajectory_.size())
            tmp_traj_index = peds_[i].trajectory_.size() - 1;
        else
            tmp_traj_index = step_;
        double vx, vy, yaw;
        vx = peds_[i].trajectory_v_[tmp_traj_index].x;
        vy = peds_[i].trajectory_v_[tmp_traj_index].y;
        yaw= atan2(vy, vx);
        Point3d p_robot_pose_(peds_[i].trajectory_[tmp_traj_index].x, peds_[i].trajectory_[tmp_traj_index].y, yaw);
        peds_[i].set_position(p_robot_pose_);
        peds_[i].vx = vx;
        peds_[i].vy = vy;
        peds_[i].update_bbox();

        eps_res_msg.peds_res[i].vs.push_back(vx);
        eps_res_msg.peds_res[i].v_ys.push_back(vy);
        geometry_msgs::Pose p_msg;
        p_msg.position.x = p_robot_pose_.x;
        p_msg.position.y = p_robot_pose_.y;
        eps_res_msg.peds_res[i].poses.push_back(p_msg);
    }
}

void ImgEnv::_step_robot(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res){
    for (int i = 0; i < req.robots.size(); i++)
    {
        comn_pkg::Agent robot_msg = req.robots[i];
        if (robot_msg.alive == true)
        {
            // step
            // TODO eps_res_msg 整成一个函数， 这一段只需要cmd这一行以及处理eps_res_msg的就够了
            bool is_arrive = robots_[i].cmd(robot_msg.v, robot_msg.w, robot_msg.v_y);
            eps_res_msg.robots_res[i].vs.push_back(robot_msg.v);
            eps_res_msg.robots_res[i].ws.push_back(robot_msg.w);
            tf::Quaternion q;
            q.setRPY(0, 0, robots_[i].robot_pose_.z);
            geometry_msgs::Pose p;
            p.position.x = robots_[i].robot_pose_.x;
            p.position.y = robots_[i].robot_pose_.y;
            p.orientation.x = q.getX();
            p.orientation.y = q.getY();
            p.orientation.z = q.getZ();
            p.orientation.w = q.getW();
            eps_res_msg.robots_res[i].poses.push_back(p);
        }

        // [robot cmd 走一步之后, base已经更新]
        double pax, pay, pbx, pby;
        robots_[i].get_corners(pax, pay, pbx, pby);
        // 对行人来说 机器人也是障碍物
        if(relation_ped_robo == 1)
            pedscene->setRobotPos(i, pax, pay, pbx, pby, robots_[i].robot_pose_.x, robots_[i].robot_pose_.y,
            robots_[i].vx, robots_[i].vy);
    }
}

bool ImgEnv::_step(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res)
{
    _step_ped(req, res);
    _step_robot(req, res);

//    vector<RVO::Vector2> next_goals;
//    for(int i = 0 ;i<peds_.size();i++){
////        if (peds_[i].cur_traj_index_ == -1 || peds_[i].trajectory_.size() == 0 || (peds_[i].arrive(peds_[i].trajectory_[-1])))
////        {
////            peds_[i].cur_traj_index_ = -1;
////            next_goals.push_back(RVO::Vector2(peds_[i].target_pose_.x, peds_[i].target_pose_.y));
////        }
////        else
////        {
//            if (peds_[i].arrive(peds_[i].trajectory_[peds_[i].cur_traj_index_]))
//                peds_[i].cur_traj_index_ += 1;
//            geometry_msgs::Point cur_ped_goal = peds_[i]._get_cur_goal();
//            next_goals.push_back(RVO::Vector2(cur_ped_goal.x, cur_ped_goal.y));
////        }
//
//    }
//
//    vector<RVO::Vector2> ps_;
//    vector<float> rs_;
//        // probability of ped respond to beep
//        for (int j = 0; j < req.robots.size(); j++)
//        {
//            bool is_beep = false;
//            robots_[j].beep = 0;
//            if (rand() / double(RAND_MAX) < ped_ca_p){
//                double beep_radius = req.robots[j].v_y;
//                if (beep_radius > 0)
//                   {
//                        Point3d rpose = robots_[j].robot_pose_;
//                        ps_.push_back(RVO::Vector2(rpose.x, rpose.y));
//                        rs_.push_back(beep_r);
//                        is_beep = true;
//                        robots_[j].beep = 1;
//                   }
//            }
//            if (is_beep == false){
//                ps_.push_back(RVO::Vector2(0, 0));
//                rs_.push_back(0.0);
//            }
//        }
//    pedscene->step(next_goals, ps_, rs_);


//    // TODO
//    for (int i = 0; i < peds_.size(); i++)
//    {
//
//        double vx, vy, yaw;
//        geometry_msgs::Pose p_msg;
//        pedscene->getNewPosAndVel(p_msg, i, vx, vy);
//        Point3d p_robot_pose_(p_msg.position.x, p_msg.position.y, yaw);
//        peds_[i].set_position(p_robot_pose_);
//        peds_[i].vx = vx;
//        peds_[i].vy = vy;
//        peds_[i].update_bbox();
//
//        eps_res_msg.peds_res[i].vs.push_back(vx);
//        eps_res_msg.peds_res[i].v_ys.push_back(vy);
//        eps_res_msg.peds_res[i].poses.push_back(p_msg);
//
//    }
//
//    for (int i = 0; i < req.robots.size(); i++)
//    {
//        comn_pkg::Agent robot_msg = req.robots[i];
//        if (robot_msg.alive == true)
//        {
//            // step
//            // TODO eps_res_msg 整成一个函数， 这一段只需要cmd这一行以及处理eps_res_msg的就够了
//            bool is_arrive = robots_[i].cmd(robot_msg.v, robot_msg.w, robot_msg.v_y);
//            eps_res_msg.robots_res[i].vs.push_back(robot_msg.v);
//            eps_res_msg.robots_res[i].ws.push_back(robot_msg.w);
//            tf::Quaternion q;
//            q.setRPY(0, 0, robots_[i].robot_pose_.z);
//            geometry_msgs::Pose p;
//            p.position.x = robots_[i].robot_pose_.x;
//            p.position.y = robots_[i].robot_pose_.y;
//            p.orientation.x = q.getX();
//            p.orientation.y = q.getY();
//            p.orientation.z = q.getZ();
//            p.orientation.w = q.getW();
//            eps_res_msg.robots_res[i].poses.push_back(p);
//        }
//
//        // [robot cmd 走一步之后, base已经更新]
//        double pax, pay, pbx, pby;
//        robots_[i].get_corners(pax, pay, pbx, pby);
//        // 对行人来说 机器人也是障碍物
//        if(relation_ped_robo == 1)
//            pedscene->setRobotPos(i, pax, pay, pbx, pby, robots_[i].robot_pose_.x, robots_[i].robot_pose_.y,
//            robots_[i].vx, robots_[i].vy);
//    }
    step_ += 1;
    view_agent();
    get_states(res.robot_states);
    if (is_show_gui_)
        show_gui();
    sleep(sleep_t_);
    return true;
}

bool ImgEnv::_end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res)
{
//    ROS_INFO("ep finish.");
    if (pub_record_)
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3,
                                        EnvMap_maps_.peds_traj_map_.map_);
        img_bridge.toImageMsg(eps_res_msg.ped_map);
    }

    for (int i = 0; i < robot_total_; i++)
        eps_res_msg.robots_res[i].result = req.robot_res[i];

    episode_res_pub_.publish(eps_res_msg);

    return true;
}

void ImgEnv::get_states(vector<comn_pkg::AgentState>& robot_states)
{
    for (int i = 0; i < robot_total_; i++)
    {
        comn_pkg::AgentState state_msg;
        vector<double> state;
        robots_[i].get_state(state);
        state_msg.state.assign(state.begin(), state.end());
        state_msg.is_collision = robots_[i].is_collision_;
        state_msg.is_arrive = robots_[i].is_arrive_;
        state_msg.laser.assign(robots_[i].hits_.begin(), robots_[i].hits_.end());
        state_msg.hits_x.assign(robots_[i].hit_points_x_.begin(), robots_[i].hit_points_x_.end());
        state_msg.hits_y.assign(robots_[i].hit_points_y_.begin(), robots_[i].hit_points_y_.end());
        state_msg.angular_map.assign(robots_[i].angular_map_.begin(), robots_[i].angular_map_.end());
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1,
                                        robots_[i].view_map_.map_);
        img_bridge.toImageMsg(state_msg.view_map);
        // add ped_states

        for(int j=0; j<ped_total_; j++){
                comn_pkg::PedInfo ped_info;
                tf::Transform tf_world_base = robots_[i].get_base_world().inverse();
                tf::Vector3 ped_pos(peds_[j].robot_pose_.x,peds_[j].robot_pose_.y,0);
                tf::Vector3 ped_pos_base=tf_world_base * ped_pos;

                tf::Vector3 ped_v(peds_[j].vx, peds_[j].vy, 0);
                tf_world_base.setOrigin(tf::Vector3(0,0,0));
                tf::Vector3 ped_v_base=tf_world_base * ped_v;
                ped_info.px=ped_pos_base.getX();
                ped_info.py=ped_pos_base.getY();

                ped_info.vx=ped_v_base.getX();
                ped_info.vy=ped_v_base.getY();
                ped_info.r_ = peds_[j].sizes_[2];
                state_msg.pedinfo.push_back(ped_info);
        }
        robot_states.push_back(state_msg);
    }
}

void ImgEnv::view_agent(){
    view_ped();
    view_robot();
}

void ImgEnv::view_ped(){
    EnvMap_maps_.peds_map_ =  EnvMap_maps_.obs_map_;
    for(int i = 0;i < peds_.size(); i++){
        string ped_name = peds_[i].name_;
        // TODO 多态
        if (peds_[i].shape_ == "circle")
        {
            peds_[i].draw(EnvMap_maps_.peds_map_, 1, "world_map", peds_[i].bbox_ );
            peds_[i].draw_rgb(EnvMap_maps_.traj_map_, colors_[7] , "world_map", peds_[i].bbox_ );
            if (pub_record_)
            {
                peds_[i].draw_rgb(EnvMap_maps_.peds_traj_map_, colors_[7] , "world_map", peds_[i].bbox_ );
            }
        }
        else if (peds_[i].shape_ == "leg")
        {
            peds_[i].draw_leg(EnvMap_maps_.peds_map_, 1);
            peds_[i].draw_leg_rgb(EnvMap_maps_.traj_map_, colors_[7]);
            if (pub_record_)
            {
                peds_[i].draw_leg_rgb(EnvMap_maps_.peds_traj_map_, colors_[7]);
            }
        }
    }
}

void ImgEnv::view_robot(){
    for (int i = 0; i < robot_total_; i++){
    // TODO: using peds_map_;
        robots_[i].global_map_ = EnvMap_maps_.peds_map_;
        for (int j = 0; j < robot_total_; j++)
        {
            if (i != j)  // 在robot i 的全局地图里 标记robot j的占位符。
                robots_[j].draw(robots_[i].global_map_, 2, "world_map", robots_[j].bbox_);
        }
        robots_[i].view(robots_[i].global_map_);
        if (i == robot_total_ - 1)
        {
            if (is_draw_step_ && (step_ % step_draw_ == 0))
            {
                for (int j = 0; j < robot_total_; j++)
                {
                    // TODO: traj_map_ no peds
                    robots_[j].draw_rgb(EnvMap_maps_.traj_map_, colors_[j], "world_map", robots_[j].bbox_);
                }
            }
            EnvMap_maps_.visual_map_ = EnvMap_maps_.traj_map_;
            vector<Point2d> head{ Point2d(0.4, 0), Point2d(0.5, 0), Point2d(0.6, 0), Point2d(0.7, 0), Point2d(0.8, 0) };
            for (int j = 0; j < robot_total_; j++)
            {
                // gui 画 beep
                if(robots_[j].beep == 1)
                    robots_[j].draw_rgb(EnvMap_maps_.visual_map_, Vec3b(179, 40, 81), "world_map", robots_[j].bbox_beep_);

                robots_[j].draw_rgb(EnvMap_maps_.visual_map_, Vec3b(0, 0, 0), "world_map", robots_[j].bbox_);
                vector<Point2d> head;
                if (robots_[j].shape_ == "circle")
                {
                    double r = robots_[j].sizes_[2];
                    int head_total = r  * 1.3 / view_resolution_;
                    for (int h = 0; h < head_total; h++)
                    {
                        head.push_back(Point2d(r + h * view_resolution_, 0));
                        head.push_back(Point2d(r + h * view_resolution_, view_resolution_));
                        head.push_back(Point2d(r + h * view_resolution_, -view_resolution_));
                    }
                }
                else if (robots_[j].shape_ == "rectangle")
                {
                    double max_x = robots_[j].sizes_[1];
                    int head_total = (robots_[j].sizes_[1] - robots_[j].sizes_[0]) / view_resolution_;
                    for (int h = 0; h < head_total; h++)
                    {
                        head.push_back(Point2d(max_x + h * view_resolution_, 0));
                    }
                }
                robots_[j].draw_rgb(EnvMap_maps_.visual_map_, Vec3b(0, 0, 0), "world_map", head);
            }
        }
    }
}

void ImgEnv::show_gui()
{
        double resize_time = double(window_height_) / EnvMap_maps_.visual_map_.img_height_;
        int window_width = int(resize_time * EnvMap_maps_.visual_map_.img_width_);
        Mat visual_image;
        resize(EnvMap_maps_.visual_map_.map_, visual_image, Size(window_width, window_height_));
        for (int i = 0; i < robot_total_; i++)
        {
            Point2i begin, end;
            EnvMap_maps_.visual_map_.world2map(Point2d(robots_[i].robot_pose_.x, robots_[i].robot_pose_.y), begin);
            EnvMap_maps_.visual_map_.world2map(Point2d(robots_[i].target_pose_.x, robots_[i].target_pose_.y), end);
            line(visual_image, Point(int(resize_time * begin.y), int(resize_time * begin.x)),
                 Point(int(resize_time * end.y), int(resize_time * end.x)), Vec3b(0,0,255), 2);
            circle(visual_image, Point(int(resize_time * end.y), int(resize_time * end.x)), int(0.2/view_resolution_*resize_time), Vec3b(255,0, 0));
            putText(visual_image, to_string(i), Point(int(resize_time * end.y)-5, int(resize_time * end.x)+5), 0, 0.5, Vec3b(255,0, 0), 2);
        }
        Mat final_image = cv::Mat::ones(window_height_, window_width + int(ceil((float(robot_total_) / show_robot_num_)))*show_image_height_, CV_8UC(3)) * 200;
        Mat roi(final_image, Rect(0, 0, window_width, window_height_));
        visual_image.copyTo(roi);
//        int show_total = min(show_robot_num_, int(envs_[m].robots_name_.size()));
        for (int i = 0; i < robot_total_; i++)
        {
            int cols = i / show_robot_num_;
            int rows = i % show_robot_num_;
            Mat roi(final_image, Rect(window_width + cols * show_image_height_, rows * show_image_height_, show_image_height_, show_image_height_));
            Mat robot_img;
            resize(robots_[i].view_map_.map_, robot_img, Size(show_image_height_, show_image_height_));
            cvtColor(robot_img, roi, COLOR_GRAY2BGR);
            rectangle(final_image, Point(window_width  + cols * show_image_height_, rows * show_image_height_),
                      Point(window_width+ (cols+1) * show_image_height_-1, (rows+1) * show_image_height_), Vec3b(255,0,0), 2);
            putText(final_image, to_string(i), Point(window_width  + cols * show_image_height_+5, (rows+1) * show_image_height_-5), 0, 0.5, Vec3b(255,0, 0), 2);
        }
        imshow(env_name, final_image);
        waitKey(1);
}





EnvService::EnvService(){
    nh_ = ros::NodeHandle("~");

    reset_env_srv = nh_.advertiseService("reset_image_env", &EnvService::reset_env, this);
    step_srv = nh_.advertiseService("step_image_env", &EnvService::step_env, this);
    init_env_srv = nh_.advertiseService("init_image_env", &EnvService::init_env, this);
    ep_end_srv = nh_.advertiseService("ep_end_image_env", &EnvService::end_ep, this);

}

bool EnvService::init_env(comn_pkg::InitEnv::Request &req, comn_pkg::InitEnv::Response &res)
{
    //for (int i=0; i < req.env.size(); i++){
    ImgEnv_env = ImgEnv();

    ImgEnv_env._init_param_from_py(req);

    ImgEnv_env._init(req.env);
        //envs_.push_back(ImgEnv_env);
    //}
    return true;
}
bool EnvService::reset_env(comn_pkg::ResetEnv::Request &req, comn_pkg::ResetEnv::Response &res)
{
//    ROS_INFO("Reset env %d", req.env_id);
    ImgEnv_env._reset(req, res);
    // view robot 问题
    return true;
}
bool EnvService::step_env(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res)
{
    ImgEnv_env._step(req, res);
    return true;
}
bool EnvService::end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res)
{
    ImgEnv_env._end_ep(req, res);
    return true;
}




ImgEnv::~ImgEnv()
{
//    for (int i = 0; i < thread_total_; i++)
//    {
//        thread_status_[i] = -1;
//        delete thread_vector_[i];
//        thread_vector_[i] = NULL;
//    }
//    for (int j = 0; j < env_total_; j++)
//    {
//        for (auto a : pedscenes_[j]->getAllAgents()) { delete a; };
//        for (auto w : pedscenes_[j]->getAllWaypoints()) { delete w; };
//        for (auto o : pedscenes_[j]->getAllObstacles()) { delete o; };
//        delete pedscenes_[j];
//        delete RVOscenes_[j];
//        delete ERVOscenes_[j];
//    }
}
