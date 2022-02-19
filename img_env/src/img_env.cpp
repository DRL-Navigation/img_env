#include "img_env.h"
#include <ros/ros.h>
#include <unistd.h>

ImgEnv::ImgEnv() : DRLConsole("img_env", boost::bind(&ImgEnv::oncmd, this, _1))
{
    ros::NodeHandle private_nh("~");
    int cpu_num = sysconf(_SC_NPROCESSORS_ONLN);
    private_nh.param("sleep_time", sleep_t_, 0.0);
    private_nh.param("max_thread", max_thread_, cpu_num);
    private_nh.param("window_height", window_height_, 500);
    private_nh.param("show_image_height", show_image_height_, 125);
    private_nh.param("show_gui", is_show_gui_, true);
    private_nh.param("is_draw_step", is_draw_step_, true);
    private_nh.param("step_draw", step_draw_, 3);
    private_nh.param("use_laser", use_laser_, true);
    private_nh.param("range_total", range_total_, 1000);
    private_nh.param("view_angle_begin", view_angle_begin_, -3.14159/2);
    private_nh.param("view_angle_end", view_angle_end_, 3.14159/2);
    private_nh.param("view_min_dist", view_min_dist_, 0.0);
    private_nh.param("view_max_dist", view_max_dist_, 10.0);
    private_nh.param("ped_consider_robot", relation_ped_robo, 1);
    std::cout<<relation_ped_robo<<std::endl;
    show_robot_num_ = int(window_height_ / show_image_height_);
    thread_total_ = 0;
    episode_res_pub_ = nh_.advertise<comn_pkg::EpRes>("/episode_res", 100);
    pub_record_ = false;
    reset_env_srv = nh_.advertiseService("/reset_image_env", &ImgEnv::reset_env, this);
    step_srv = nh_.advertiseService("/step_image_env", &ImgEnv::step_env, this);
    init_env_srv = nh_.advertiseService("/init_image_env", &ImgEnv::init_env, this);
    ep_end_srv = nh_.advertiseService("/ep_end_image_env", &ImgEnv::end_ep, this);
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
    ROS_INFO("Wait for the python environment to initialize!");
}

void ImgEnv::oncmd(const char *cmd)
{
    float f1, f2, f3, f4, f5, f6, f7, f8, f9, f10;
    int d1, d2;
    char str1[50] = "";
    char str2[50] = "";
    char str3[50] = "";
    char str4[50] = "";
    if (PEEK_CMD(cmd, "show"))
    {
        if (is_show_gui_)
            is_show_gui_ = false;
        else
            is_show_gui_ = true;
    }
    else if (PEEK_CMD(cmd, "record"))
    {
        pub_record_ = true;
    }
}

bool ImgEnv::init_env(comn_pkg::InitEnv::Request &req, comn_pkg::InitEnv::Response &res)
{
    if (thread_total_ != 0)
    {
        ROS_ERROR("Restart me !");
        exit(0);
    }
    step_ = 0;
    robots_.clear();
    env_total_ = req.envs.size();
    envs_.clear();
    envs_index_.clear();
    view_resolution_ = req.view_resolution;
    for (int i = 0; i < env_total_; i++)
    {
        Ped::Tscene *pedscene = new Ped::Tscene(0,10,10,10);
        RVO::RVOSimulator *rvo_sim = new RVO::RVOSimulator();
        RVO::ERVOSimulator *ervo_sim = new RVO::ERVOSimulator();
        // todo add boudaries
        pedscenes_.push_back(pedscene);
        RVOscenes_.push_back(rvo_sim);
        ERVOscenes_.push_back(ervo_sim);
        EnvInfo env;
        env.name_ = req.envs[i].name;
        env.static_map_.resolution_ = req.envs[i].global_resolution;
        env.static_map_.read_image(req.envs[i].map_file, view_resolution_);
        env.obs_map_ = env.static_map_;

        for (int j = 0; j < req.envs[i].robots.size(); j++)
        {
            Agent robot(req.envs[i].robots[j].ktype);
            robot.step_hz_ = req.step_hz;
            step_hz_ = req.step_hz;
            robot.state_dim_ = req.state_dim;
            robot.init_view_map(req.view_width, req.view_height, view_resolution_);
            vector<double> rob_size;
            for (int k = 0; k < req.envs[i].robots[j].size.size(); k++)
                rob_size.push_back(req.envs[i].robots[j].size[k]);
            robot.init_shape(req.envs[i].robots[j].shape, rob_size);
            robot.name_ = req.envs[i].robots[j].name;
            robot.use_laser_ = use_laser_;
            robot.range_total_ = range_total_;
            robot.view_angle_begin_ = view_angle_begin_;
            robot.view_angle_end_ = view_angle_end_;
            robot.view_min_dist_ = view_min_dist_;
            robot.view_max_dist_ = view_max_dist_;
            robot.sensor_base_.x = req.envs[i].robots[j].sensor_cfg[0];
            robot.sensor_base_.y = req.envs[i].robots[j].sensor_cfg[1];
            robots_.push_back(robot);
            Ped::Tagent* robot_sim = new Ped::Tagent();
            if(relation_ped_robo==1){
                pedscene->addAgent(robot_sim);
                rvo_sim->addAgent(RVO::Vector2(0, 0),3, 10, 5, 5, 0.5, 0.6);
                ervo_sim->addAgent(RVO::Vector2(0, 0), 3, 10, 5, 5, 0.5, 0.6);
                rvo_index_.insert(pair<string, int>(robot.name_, rvo_sim->getNumAgents()-1));
            }
            robots_sim_.push_back(robot_sim);
            env.robots_name_.push_back(robot.name_);
            robots_envs_.insert(pair<string, string>(robot.name_, env.name_));
            robots_index_.insert(pair<string, int>(robot.name_, robots_.size()-1));
        }
        rvo_sim->setTimeStep(step_hz_);
        ervo_sim->setTimeStep(step_hz_);
        for (int k = 0; k < req.envs[i].peds.size(); k++)
        {
            PedAgent ped(req.envs[i].peds[k].ktype);
            ped.step_hz_ = req.step_hz;
            step_hz_ = req.step_hz;
            vector<double> ped_size;
            for (int m = 0; m < req.envs[i].peds[k].size.size(); m++)
                ped_size.push_back(req.envs[i].peds[k].size[m]);
            ped.init_shape(req.envs[i].peds[k].shape, ped_size);
            ped.name_ = req.envs[i].peds[k].name;
            peds_.push_back(ped);
            Ped::Tagent* ped_tmp = new Ped::Tagent();
            double pxx = rand()/2147483647.0*10.0;
            double pyy = rand()/2147483647.0*10.0;
            ped_tmp->setPosition(pxx, pyy, 0);
            // 行人最大速度
            double ped_max_speed = 0.5;
            ped_tmp->setVmax(ped_max_speed);
            pedscenes_[i]->addAgent(ped_tmp);
            peds_sim_.push_back(ped_tmp);
            env.peds_name_.push_back(ped.name_);
            peds_envs_.insert(pair<string, string>(ped.name_, env.name_));
            peds_index_.insert(pair<string, int>(ped.name_, peds_.size()-1));

            rvo_sim->addAgent(RVO::Vector2(pxx, pyy), 3 ,10, 5, 5, 0.5, ped_max_speed);
            ervo_sim->addAgent(RVO::Vector2(pxx, pyy), 3, 10, 5, 5, 0.5, ped_max_speed);
            rvo_index_.insert(pair<string, int>(ped.name_, rvo_sim->getNumAgents()-1));
        }
        envs_.push_back(env);
        envs_index_.insert(pair<string, int>(env.name_, envs_.size()-1));
    }
    robot_total_ = robots_.size();

    thread_total_ = int(min(robot_total_, max_thread_));
    cout << "init thread: " << thread_total_<< endl;
    thread_vector_.resize(thread_total_);
    thread_status_.resize(thread_total_);
    int robot_split = std::ceil(robot_total_ / double(thread_total_));
    for (int i = 0; i < thread_total_; i++)
    {
        int begin_index = robot_split * i;
        int end_index = std::min(robot_split * (i+1), robot_total_);
        thread_status_[i] = 0;
        thread_vector_[i] = new boost::thread(boost::bind(&ImgEnv::view_thread, this,
                                                          begin_index, end_index, i));
    }
    return true;
}

bool ImgEnv::reset_env(comn_pkg::ResetEnv::Request &req, comn_pkg::ResetEnv::Response &res)
{
//    ROS_INFO("reset env ...");
    eps_res_msg.clear();
    eps_res_msg.resize(env_total_);
    for (int i = 0; i < env_total_; i++)
    {
        envs_[i].obs_map_ = envs_[i].static_map_;
        RVOscenes_[i]->clearObstacle();
        ERVOscenes_[i]->clearObstacle();
    }
    // clean up all obstacles in pedsim
    // for (int i = 0; i < obs_sim_.size(); i++)
    // {
    //     delete obs_sim_[i];
    // }
//    for (int i = 0; i < peds_.size(); i++){
//        peds_[i].reset();
//    }
    for (int i = 0; i < req.obstacles.size(); i++)
    {
        comn_pkg::Agent obs_msg = req.obstacles[i];
        Agent obs("obs", view_resolution_);
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
        obs.draw(envs_[envs_index_[obs_msg.env_name]].obs_map_, 0, "world_map", obs.bbox_);

        // init obstacles in pedsim scenes
        double pax, pay, pbx, pby;
        obs.get_corners(pax, pay, pbx, pby);
        Ped::Tobstacle *pedobs = new Ped::Tobstacle(pax, pay, pbx, pby);
        if(obs_sim_.size()!=req.obstacles.size()){
            obs_sim_.push_back(pedobs);
            pedscenes_[envs_index_[obs_msg.env_name]]->addObstacle(pedobs);
        }
        else
            obs_sim_[i]->setPosition(pax, pay, pbx, pby);

        std::vector<RVO::Vector2> obs_shape;
        obs_shape.push_back(RVO::Vector2(pax, pay));
        obs_shape.push_back(RVO::Vector2(pax, pby));
        obs_shape.push_back(RVO::Vector2(pbx, pby));
        obs_shape.push_back(RVO::Vector2(pbx, pay));
        RVOscenes_[envs_index_[obs_msg.env_name]]->addObstacle(obs_shape);
        ERVOscenes_[envs_index_[obs_msg.env_name]]->addObstacle(obs_shape);
    }
    vector<int> env_robots_count;
    vector<int> env_peds_count;
    for (int i = 0; i < env_total_; i++)
    {
        env_robots_count.push_back(0);
        env_peds_count.push_back(0);
        envs_[i].traj_map_ = envs_[i].obs_map_;
        cvtColor(envs_[i].traj_map_.map_, envs_[i].traj_map_.map_, COLOR_GRAY2BGR);
        if (pub_record_)
        {
            envs_[i].peds_traj_map_ = envs_[i].obs_map_;
            cvtColor(envs_[i].peds_traj_map_.map_, envs_[i].peds_traj_map_.map_, COLOR_GRAY2BGR);
        }
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1,
                                        envs_[i].obs_map_.map_);
        img_bridge.toImageMsg(eps_res_msg[i].obs_map);
        eps_res_msg[i].env_name = envs_[i].name_;
        eps_res_msg[i].resolution = view_resolution_;
        eps_res_msg[i].step_hz = step_hz_;
        eps_res_msg[i].robots_res.resize(int(envs_[i].robots_name_.size()));
        eps_res_msg[i].peds_res.resize(int(envs_[i].peds_name_.size()));
    }
    for (int i = 0; i < req.robots.size(); i++)
    {
        string robot_name = req.robots[i].name;
        string env_name = robots_envs_[robot_name];
        int env_index = envs_index_[env_name];
        int robot_index = env_robots_count[env_index];
        eps_res_msg[env_index].robots_res[robot_index].info = req.robots[i];
        eps_res_msg[env_index].robots_res[robot_index].poses.clear();
        eps_res_msg[env_index].robots_res[robot_index].vs.clear();
        eps_res_msg[env_index].robots_res[robot_index].ws.clear();
        eps_res_msg[env_index].robots_res[robot_index].v_ys.clear();
        env_robots_count[env_index] += 1;
        comn_pkg::Agent robot_msg = req.robots[i];
        eps_res_msg[env_index].robots_res[robot_index].poses.push_back(robot_msg.init_pose);
        tf::Quaternion q(robot_msg.init_pose.orientation.x, robot_msg.init_pose.orientation.y,
                         robot_msg.init_pose.orientation.z, robot_msg.init_pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        Point3d rob_p(robot_msg.init_pose.position.x, robot_msg.init_pose.position.y, yaw);
        robots_[i].init_pose(rob_p);
        Point3d rob_goal(robot_msg.goal.x, robot_msg.goal.y, 0);
        robots_[i].set_goal(rob_goal);

        // change position of robots in pedsim 
        double pax, pay, pbx, pby;
        robots_[i].get_corners(pax, pay, pbx, pby);
        // 对行人来说 机器人也是障碍物 
        robots_sim_[i]->setPosition(robot_msg.init_pose.position.x,robot_msg.init_pose.position.y,0);

        if(relation_ped_robo==1){
            RVO::ERVOSimulator * ervos = ERVOscenes_[env_index];
            ervos->setAgentPosition(rvo_index_[robot_name], RVO::Vector2(robot_msg.init_pose.position.x, robot_msg.init_pose.position.y));
            ervos->setAgentVelocity(rvo_index_[robot_name], RVO::Vector2(0, 0));

            RVO::RVOSimulator * rvos = RVOscenes_[env_index];
            rvos->setAgentPosition(rvo_index_[robot_name], RVO::Vector2(robot_msg.init_pose.position.x, robot_msg.init_pose.position.y));
            rvos->setAgentVelocity(rvo_index_[robot_name], RVO::Vector2(0, 0));
        }
    }

    for (int i = 0; i < env_total_; i++)
    {
        RVOscenes_[i]->processObstacles();
        ERVOscenes_[i]->processObstacles();
    }

    for (int i = 0; i < req.peds.size(); i++)
    {   

        string ped_name = req.peds[i].name;
        string env_name = peds_envs_[ped_name];
        int env_index = envs_index_[env_name];
        int ped_index = env_peds_count[env_index];

        eps_res_msg[env_index].peds_res[ped_index].info = req.peds[i];
        eps_res_msg[env_index].peds_res[ped_index].poses.clear();
        eps_res_msg[env_index].peds_res[ped_index].vs.clear();
        eps_res_msg[env_index].peds_res[ped_index].v_ys.clear();

        env_peds_count[env_index] += 1;
        
        comn_pkg::Agent ped_msg = req.peds[i];
        eps_res_msg[env_index].peds_res[ped_index].poses.push_back(ped_msg.init_pose);

        tf::Quaternion q(ped_msg.init_pose.orientation.x, ped_msg.init_pose.orientation.y,
                         ped_msg.init_pose.orientation.z, ped_msg.init_pose.orientation.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // 这里都是世界坐标系

        Point3d ped_p(ped_msg.init_pose.position.x, ped_msg.init_pose.position.y, yaw);    
        peds_[i].init_pose(ped_p);
          
        Point3d ped_goal(ped_msg.goal.x, ped_msg.goal.y, 0);

        peds_[i].set_goal(ped_goal);

        // change position and goal of peds in pedsim
        peds_sim_[i]->setPosition(ped_p.x, ped_p.y, 0);
        // delete waypoint

        peds_[i].trajectory_.clear();
        peds_[i].trajectory_ = ped_msg.trajectory;
        peds_[i].cur_traj_index_ = 0;
        peds_sim_[i]->clearWaypoints();
        for (int k = 0; k < ped_msg.trajectory.size(); k++)
        {
            Ped::Twaypoint *wp = new Ped::Twaypoint(ped_msg.trajectory[k].x, ped_msg.trajectory[k].y, ped_msg.trajectory[k].z);
            peds_sim_[i]->addWaypoint(wp);
        }
        Ped::Twaypoint *w_goal = new Ped::Twaypoint(ped_goal.x, ped_goal.y, 1);
        peds_sim_[i]->addWaypoint(w_goal);
        // add init point to waypoint
//        peds_sim_[i]->addWaypoint(new Ped::Twaypoint(ped_msg.init_pose.position.x, ped_msg.init_pose.position.y,1));

        RVOscenes_[env_index]->setAgentPosition(rvo_index_[ped_name], RVO::Vector2(ped_msg.init_pose.position.x, ped_msg.init_pose.position.y));
        ERVOscenes_[env_index]->setAgentPosition(rvo_index_[ped_name], RVO::Vector2(ped_msg.init_pose.position.x, ped_msg.init_pose.position.y));
    }
    get_states(res.robot_states);

    if (is_show_gui_)
        show_gui();
    return true;
}

bool ImgEnv::step_env(comn_pkg::StepEnv::Request &req, comn_pkg::StepEnv::Response &res)
{
    RVO::Vector2 next_goal;
    for (int i = 0; i <peds_.size(); i++)
    {
        if (peds_[i].cur_traj_index_ == -1 || peds_[i].trajectory_.size() == 0 || (peds_[i].arrive(peds_[i].trajectory_[-1])))
        {
            peds_[i].cur_traj_index_ = -1;
            next_goal = RVO::Vector2(peds_[i].target_pose_.x, peds_[i].target_pose_.y);
        }
        else
        {
            if (peds_[i].arrive(peds_[i].trajectory_[peds_[i].cur_traj_index_]))
                peds_[i].cur_traj_index_ += 1;
            next_goal = RVO::Vector2(peds_[i].trajectory_[peds_[i].cur_traj_index_].x, peds_[i].trajectory_[peds_[i].cur_traj_index_].y);
        }
        RVO::RVOSimulator * rvos = RVOscenes_[envs_index_[peds_envs_[peds_[i].name_]]];
        RVO::Vector2 goalVector = next_goal - rvos->getAgentPosition(rvo_index_[peds_[i].name_]);
        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector);
        }
        rvos->setAgentPrefVelocity(rvo_index_[peds_[i].name_], goalVector);

        RVO::ERVOSimulator * ervos = ERVOscenes_[envs_index_[peds_envs_[peds_[i].name_]]];
        goalVector = RVO::Vector2(peds_[i].target_pose_.x, peds_[i].target_pose_.y)
                - ervos->getAgentPosition(rvo_index_[peds_[i].name_]);
        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector);
        }
        ervos->setAgentPrefVelocity(rvo_index_[peds_[i].name_], goalVector);
    }
    vector<int> env_robots_count;
    vector<int> env_peds_count;
    for (int i = 0; i < env_total_; i++)
    {
        env_robots_count.push_back(0);
        env_peds_count.push_back(0);
    }

    for (int i = 0; i < pedscenes_.size(); i++)
    {
        // pedsim
        pedscenes_[i]->moveAgents(step_hz_);
        // rvo
        RVOscenes_[i]->doStep();
        // ervo
        vector<string> env_robot_names = envs_[i].robots_name_;

        bool is_beep = false;
        for (int j = 0; j < env_robot_names.size(); j++)
        {
            double beep_radius = req.robots[robots_index_[env_robot_names[j]]].v_y;
            if (beep_radius > 0)
            {
                Point3d rpose = robots_[robots_index_[env_robot_names[j]]].robot_pose_;
                ERVOscenes_[i]->doStep(RVO::Vector2(rpose.x, rpose.y), beep_radius);
                is_beep = true;
                break;
            }

        }
        if (is_beep == false)
            ERVOscenes_[i]->doStep(RVO::Vector2(0, 0), 0);
    }
    for (int i = 0; i < peds_.size(); i++)
    {
        string ped_name = peds_[i].name_;
        string env_name = peds_envs_[ped_name];
        int env_index = envs_index_[env_name];
        int ped_index = env_peds_count[env_index];
        double vx, vy;
        geometry_msgs::Pose p_msg;
        // 所有行人走一步后,得到新的位置,(世界坐标系) 赋给img_env 对应 行人agent的位置
        if (peds_[i].ktype_ == "pedsim")
        {
            Ped::Tvector p = peds_sim_[i]->getPosition();
            vx = peds_sim_[i]->getVelocity().x;
            vy = peds_sim_[i]->getVelocity().y;
            double yaw= atan2(vy, vx);
            Point3d p_robot_pose_(p.x, p.y, yaw);
            peds_[i].set_position(p_robot_pose_);
            peds_[i].vx = vx;
            peds_[i].vy = vy;
            p_msg.position.x = p.x;
            p_msg.position.y = p.y;
            p_msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else if (peds_[i].ktype_ == "rvo")
        {
            RVO::RVOSimulator * rvos = RVOscenes_[envs_index_[peds_envs_[peds_[i].name_]]];
            RVO::Vector2 p = rvos->getAgentPosition(rvo_index_[peds_[i].name_]);
            RVO::Vector2 vel = rvos->getAgentVelocity(rvo_index_[peds_[i].name_]);
            vx = vel.x();
            vy = vel.y();
            double yaw= atan2(vy, vx);
            Point3d p_pose(p.x(), p.y(), yaw);
            peds_[i].set_position(p_pose);
            peds_[i].vx = vx;
            peds_[i].vy = vy;
            p_msg.position.x = p.x();
            p_msg.position.y = p.y();
            p_msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else if (peds_[i].ktype_ == "ervo")
        {
            RVO::ERVOSimulator * ervos = ERVOscenes_[envs_index_[peds_envs_[peds_[i].name_]]];
            RVO::Vector2 p = ervos->getAgentPosition(rvo_index_[peds_[i].name_]);
            RVO::Vector2 vel = ervos->getAgentVelocity(rvo_index_[peds_[i].name_]);
            vx = vel.x();
            vy = vel.y();
            double yaw= atan2(vy, vx);
            Point3d p_pose(p.x(), p.y(), yaw);
            peds_[i].set_position(p_pose);
            peds_[i].vx = vx;
            peds_[i].vy = vy;
            p_msg.position.x = p.x();
            p_msg.position.y = p.y();
            p_msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        peds_[i].update_bbox();
        eps_res_msg[env_index].peds_res[ped_index].vs.push_back(vx);
        eps_res_msg[env_index].peds_res[ped_index].v_ys.push_back(vy);
        eps_res_msg[env_index].peds_res[ped_index].poses.push_back(p_msg);
        env_peds_count[env_index] += 1;
    }
    
    for (int i = 0; i < req.robots.size(); i++)
    {
        string robot_name = req.robots[i].name;
        string env_name = robots_envs_[robot_name];
        int env_index = envs_index_[env_name];
        int robot_index = env_robots_count[env_index];
        comn_pkg::Agent robot_msg = req.robots[i];
        if (robot_msg.alive == true)
        {
            // step
            bool is_arrive = robots_[i].cmd(robot_msg.v, robot_msg.w,robot_msg.v_y);
            eps_res_msg[env_index].robots_res[robot_index].vs.push_back(robot_msg.v);
            eps_res_msg[env_index].robots_res[robot_index].v_ys.push_back(req.robots[i].v_y);
            eps_res_msg[env_index].robots_res[robot_index].ws.push_back(robot_msg.w);
            tf::Quaternion q;
            q.setRPY(0, 0, robots_[i].robot_pose_.z);
            geometry_msgs::Pose p;
            p.position.x = robots_[i].robot_pose_.x;
            p.position.y = robots_[i].robot_pose_.y;
            p.orientation.x = q.getX();
            p.orientation.y = q.getY();
            p.orientation.z = q.getZ();
            p.orientation.w = q.getW();
            eps_res_msg[env_index].robots_res[robot_index].poses.push_back(p);
        }
        env_robots_count[env_index] += 1;

        // [robot cmd 走一步之后, base已经更新]

        double pax, pay, pbx, pby;
        int r_index = robots_index_[robot_name];
        robots_[r_index].get_corners(pax, pay, pbx, pby);

        if(relation_ped_robo==1){
            robots_sim_[i]->setPosition(robots_[i].robot_pose_.x, robots_[i].robot_pose_.y,0);
            RVO::ERVOSimulator * ervos = ERVOscenes_[env_index];
            ervos->setAgentPosition(rvo_index_[robot_name], RVO::Vector2(robots_[i].robot_pose_.x, robots_[i].robot_pose_.y));
            ervos->setAgentVelocity(rvo_index_[robot_name], RVO::Vector2(0, 0));

            RVO::ERVOSimulator * rvos = ERVOscenes_[env_index];
            rvos->setAgentPosition(rvo_index_[robot_name], RVO::Vector2(robots_[i].robot_pose_.x, robots_[i].robot_pose_.y));
            rvos->setAgentVelocity(rvo_index_[robot_name], RVO::Vector2(0, 0));
        }
    }

    step_ += 1;
    get_states(res.robot_states);
    if (is_show_gui_)
        show_gui();
    usleep(int(sleep_t_ * 1000000));
    return true;
}

bool ImgEnv::end_ep(comn_pkg::EndEp::Request &req, comn_pkg::EndEp::Response &res)
{
//    ROS_INFO("ep finish.");
    vector<int> env_robots_count;
    for (int i = 0; i < env_total_; i++)
    {
        env_robots_count.push_back(0);
        if (pub_record_)
        {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3,
                                            envs_[i].peds_traj_map_.map_);
            img_bridge.toImageMsg(eps_res_msg[i].ped_map);
        }
    }
    for (int i = 0; i < req.robot_res.size(); i++)
    {
        string robot_name = robots_[i].name_;
        string env_name = robots_envs_[robot_name];
        int env_index = envs_index_[env_name];
        int robot_index = env_robots_count[env_index];
        eps_res_msg[env_index].robots_res[robot_index].result = req.robot_res[i];
        env_robots_count[env_index] += 1;
    }
    for (int i = 0; i < env_total_; i++)
    {
        episode_res_pub_.publish(eps_res_msg[i]);
    }
    return true;
}

void ImgEnv::get_states(vector<comn_pkg::AgentState>& robot_states)
{
    robot_states.clear();

    // draw ped map
    
    view_ped();
    for (int i = 0; i < thread_total_; i++)
    {
        thread_status_[i] = 1;
    }
    while (true) {
        int finish_num = 0;
        for (int i = 0; i < thread_total_; i++)
        {
            if (thread_status_[i] == 0)
            {
                finish_num += 1;
            }
        }
        if (finish_num == thread_total_)
            break;
    }
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

        for(int j=0; j<peds_sim_.size();j++){
            if(peds_envs_[peds_[j].name_]==robots_envs_[robots_[i].name_]){

                comn_pkg::PedInfo ped_info;
                tf::Transform tf_world_base = robots_[i].get_base_world().inverse();
                tf::Vector3 ped_pos(peds_[j].robot_pose_.x,peds_[j].robot_pose_.y,0);
                tf::Vector3 ped_pos_base=tf_world_base * ped_pos;

                tf::Vector3 ped_v(peds_[j].vx,peds_[j].vy,0);
                tf_world_base.setOrigin(tf::Vector3(0,0,0));
                tf::Vector3 ped_v_base=tf_world_base * ped_v;
                ped_info.px=ped_pos_base.getX();
                ped_info.py=ped_pos_base.getY();
                ped_info.vx=ped_v_base.getX();
                ped_info.vy=ped_v_base.getY();
                ped_info.r_ = peds_[j].sizes_[2];

                state_msg.pedinfo.push_back(ped_info);
            }
        }

        robot_states.push_back(state_msg);
    }
    
}

void ImgEnv::view_ped(){
    for (int i=0; i<env_total_ ;i++){
        envs_[i].peds_map_=envs_[i].obs_map_;
    }
    for(int i = 0;i < peds_.size(); i++){
        string ped_name = peds_[i].name_;
        string env_name = peds_envs_[ped_name];
        int env_id = envs_index_[env_name];
        if (peds_[i].shape_ == "circle")
        {
            peds_[i].draw(envs_[env_id].peds_map_, 0, "world_map", peds_[i].bbox_ );
            peds_[i].draw_rgb(envs_[env_id].traj_map_, colors_[7] , "world_map", peds_[i].bbox_ );
            if (pub_record_)
            {
                peds_[i].draw_rgb(envs_[env_id].peds_traj_map_, colors_[7] , "world_map", peds_[i].bbox_ );
            }
        }
        else if (peds_[i].shape_ == "leg")
        {
            peds_[i].draw_leg(envs_[env_id].peds_map_, 0);
            peds_[i].draw_leg_rgb(envs_[env_id].traj_map_, colors_[7]);
            if (pub_record_)
            {
                peds_[i].draw_leg_rgb(envs_[env_id].peds_traj_map_, colors_[7]);
            }
        }
    }
//    for (int i=0; i<env_total_ ;i++){
////        envs_[i].traj_map_=envs_[i].peds_map_;
//        cvtColor(envs_[i].traj_map_.map_, envs_[i].traj_map_.map_, COLOR_GRAY2BGRA);
//    }
}

void ImgEnv::view_thread(int begin_index, int end_index, int thread_num)
{
    ros::Rate r(200);
    while(thread_status_[thread_num] != -1)
    {
        if (thread_status_[thread_num] == 1)
        {
            for (int i = begin_index; i < end_index; i++)
            {
                int env_id = envs_index_[robots_envs_[robots_[i].name_]];
                // TODO: using peds_map_;
                robots_[i].global_map_ = envs_[env_id].peds_map_;
                int env_robots = envs_[env_id].robots_name_.size();
                int last_env_robot = robots_index_[envs_[env_id].robots_name_[env_robots-1]];
                for (int j = 0; j < env_robots; j++)
                {
                    int j_index = robots_index_[envs_[env_id].robots_name_[j]];
                    if (i != j_index)
                    {
                        robots_[j_index].draw(robots_[i].global_map_, 0, "world_map", robots_[j_index].bbox_);
                    }
                }
                robots_[i].view(robots_[i].global_map_);
//                if (i == last_env_robot)
//                {
//                    vector<Point2d> head{ Point2d(0.4, 0), Point2d(0.5, 0), Point2d(0.6, 0), Point2d(0.7, 0), Point2d(0.8, 0) };
//                    envs_[env_id].visual_map_ = robots_[i].global_map_;
//                    robots_[i].draw(envs_[env_id].visual_map_, 0, "world_map", robots_[i].bbox_);
//                    for (int k = 0; k < envs_[env_id].robots_name_.size(); k++)
//                    {
//                        int k_index = robots_index_[envs_[env_id].robots_name_[k]];
//                        robots_[k_index].draw(envs_[env_id].visual_map_, 0, "world_map", head);
//                    }
//                }

                if (i == last_env_robot)
                {
                    if (is_draw_step_ && (step_ % step_draw_ == 0))
                    {
                        for (int j = 0; j < env_robots; j++)
                        {
                            int j_index = robots_index_[envs_[env_id].robots_name_[j]];
                            // TODO: traj_map_ no peds
                            robots_[j_index].draw_rgb(envs_[env_id].traj_map_, colors_[j], "world_map", robots_[j_index].bbox_);
                        }
                    }
                    envs_[env_id].visual_map_ = envs_[env_id].traj_map_;
                    vector<Point2d> head{ Point2d(0.4, 0), Point2d(0.5, 0), Point2d(0.6, 0), Point2d(0.7, 0), Point2d(0.8, 0) };

                    for (int j = 0; j < env_robots; j++)
                    {
                        int j_index = robots_index_[envs_[env_id].robots_name_[j]];
                        robots_[j_index].draw_rgb(envs_[env_id].visual_map_, Vec3b(0, 0, 0), "world_map", robots_[j_index].bbox_);
                        vector<Point2d> head;
                        if (robots_[j_index].shape_ == "circle")
                        {
                            double r = robots_[j_index].sizes_[2];
                            int head_total = r  * 1.3 / view_resolution_;
                            for (int h = 0; h < head_total; h++)
                            {
                                head.push_back(Point2d(r + h * view_resolution_, 0));
                                head.push_back(Point2d(r + h * view_resolution_, view_resolution_));
                                head.push_back(Point2d(r + h * view_resolution_, -view_resolution_));
                            }
                        }
                        else if (robots_[j_index].shape_ == "rectangle")
                        {
                            double max_x = robots_[j_index].sizes_[1];
                            int head_total = (robots_[j_index].sizes_[1] - robots_[j_index].sizes_[0]) / view_resolution_;
                            for (int h = 0; h < head_total; h++)
                            {
                                head.push_back(Point2d(max_x + h * view_resolution_, 0));
                            }
                        }
                        robots_[j_index].draw_rgb(envs_[env_id].visual_map_, Vec3b(0, 0, 0), "world_map", head);
                    }
                }
            }
            thread_status_[thread_num] = 0;
        }
        r.sleep();
    }
}

void ImgEnv::show_gui()
{
    for (int m = 0; m < env_total_; m++)
    {
        double resize_time = double(window_height_) / envs_[m].visual_map_.img_height_;
        int window_width = int(resize_time * envs_[m].visual_map_.img_width_);
        Mat visual_image;
        resize(envs_[m].visual_map_.map_, visual_image, Size(window_width, window_height_));
        for (int i = 0; i < envs_[m].robots_name_.size(); i++)
        {
            int i_index = robots_index_[envs_[m].robots_name_[i]];
            Point2i begin, end;
            envs_[m].visual_map_.world2map(Point2d(robots_[i_index].robot_pose_.x, robots_[i_index].robot_pose_.y), begin);
            envs_[m].visual_map_.world2map(Point2d(robots_[i_index].target_pose_.x, robots_[i_index].target_pose_.y), end);
            line(visual_image, Point(int(resize_time * begin.y), int(resize_time * begin.x)),
                 Point(int(resize_time * end.y), int(resize_time * end.x)), Vec3b(0,0,255), 2);
            circle(visual_image, Point(int(resize_time * end.y), int(resize_time * end.x)), int(0.2/view_resolution_*resize_time), Vec3b(255,0, 0));
            putText(visual_image, to_string(i), Point(int(resize_time * end.y)-5, int(resize_time * end.x)+5), 0, 0.5, Vec3b(255,0, 0), 2);
        }
        int robot_total = int(envs_[m].robots_name_.size());
        Mat final_image = cv::Mat::ones(window_height_, window_width + int(ceil((float(robot_total) / show_robot_num_)))*show_image_height_, CV_8UC(3)) * 200;
        Mat roi(final_image, Rect(0, 0, window_width, window_height_));
        visual_image.copyTo(roi);
//        int show_total = min(show_robot_num_, int(envs_[m].robots_name_.size()));
        for (int i = 0; i < robot_total; i++)
        {
            int cols = i / show_robot_num_;
            int rows = i % show_robot_num_;
            int i_index = robots_index_[envs_[m].robots_name_[i]];
            Mat roi(final_image, Rect(window_width + cols * show_image_height_, rows * show_image_height_, show_image_height_, show_image_height_));
            Mat robot_img;
            resize(robots_[i_index].view_map_.map_, robot_img, Size(show_image_height_, show_image_height_));
            cvtColor(robot_img, roi, COLOR_GRAY2BGR);
            rectangle(final_image, Point(window_width  + cols * show_image_height_, rows * show_image_height_),
                      Point(window_width+ (cols+1) * show_image_height_-1, (rows+1) * show_image_height_), Vec3b(255,0,0), 2);
            putText(final_image, to_string(i), Point(window_width  + cols * show_image_height_+5, (rows+1) * show_image_height_-5), 0, 0.5, Vec3b(255,0, 0), 2);
        }
        imshow(envs_[m].name_, final_image);
        waitKey(1);
    }
}

ImgEnv::~ImgEnv()
{
    for (int i = 0; i < thread_total_; i++)
    {
        thread_status_[i] = -1;
        delete thread_vector_[i];
        thread_vector_[i] = NULL;
    }
    for (int j = 0; j < env_total_; j++)
    {
        for (auto a : pedscenes_[j]->getAllAgents()) { delete a; };
        for (auto w : pedscenes_[j]->getAllWaypoints()) { delete w; };
        for (auto o : pedscenes_[j]->getAllObstacles()) { delete o; };
        delete pedscenes_[j];
        delete RVOscenes_[j];
        delete ERVOscenes_[j];
    }
}
