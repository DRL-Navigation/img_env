
#ifndef   _pedscene_   
#define   _pedscene_
#include "scene.h"
#include <pedsimros/ped_scene.h>
#include <pedsimros/ped_agent.h>
#include <pedsimros/ped_obstacle.h>
#include <pedsimros/ped_waypoint.h>
#include <ervo_ros/RVO.h>
class PedScene:public Scene{
    private: 
        Ped::Tscene* pedscene_;
        vector<Ped::Tagent *> robots_sim_;
        vector<Ped::Tobstacle *> obs_sim_;
        vector<Ped::Tagent *> peds_sim_;
    public:

        PedScene(map<string, double> params_double, map<string, vector<double>> params_vecdouble):Scene(params_double, params_vecdouble){
            pedscene_ = new Ped::Tscene(0,10,10,10);
        };
        void processObs(){};
        // virtual void step();
        void addObs(double pax, double pay, double pbx, double pby){
            Ped::Tobstacle *pedobs = new Ped::Tobstacle(pax, pay, pbx, pby);
            pedscene_->addObstacle(pedobs);
            obs_sim_.push_back(pedobs);
        };
        void clearObs(){
            for (int i=0; i< obs_sim_.size(); i++){
                pedscene_->removeObstacle(obs_sim_[i]);
            }
            obs_sim_.clear();
        };

        void setPedPos(int index,double x,double y){
            peds_sim_[index]->setPosition(x, y, 0);
        };

        void setWayPoint(int index, vector<geometry_msgs::Point> traj, double px,  double py){
            peds_sim_[index]->clearWaypoints();
            Ped::Twaypoint *wp = new Ped::Twaypoint(px, py, 1);
            peds_sim_[index]->addWaypoint(wp);
            for (int k = 0; k < traj.size(); k++){
                Ped::Twaypoint *wp = new Ped::Twaypoint(traj[k].x, traj[k].y, traj[k].z);
                peds_sim_[index]->addWaypoint(wp);
            }
        }

        void step(vector<RVO::Vector2> nextgoals, vector<RVO::Vector2> ps_, vector<float> rs_){
            pedscene_->moveAgents(step_hz_);
        }

        void setRobotPos(int index,double pax, double pay, double pbx, double pby,
                     double px,  double py, double vx, double vy){
            robots_sim_[index]->setPosition(px, py, 1);
        };

        void addPed(int ped_num, vector<double> ped_max_speed_){
            for(int i = 0 ; i <ped_num; i++){
                Ped::Tagent* ped_tmp = new Ped::Tagent();
                double pxx = rand()/2147483647.0*10.0;
                double pyy = rand()/2147483647.0*10.0;
                ped_tmp->setPosition(pxx, pyy, 0);
                // 行人最大速度
                // this may have diff
                ped_tmp->setVmax(ped_max_speed_[i]);
                pedscene_-> addAgent(ped_tmp);
                peds_sim_.push_back(ped_tmp);
            }
        };

        void addRobot(int robo_num){
            for (int i = 0; i < robo_num; i++){
            
                Ped::Tagent* robot_sim = new Ped::Tagent();
                if(relation_ped_robo==1)
                    pedscene_->addAgent(robot_sim);

                robots_sim_.push_back(robot_sim);
            }
        };
       
        void getNewPosAndVel(geometry_msgs::Pose &p_msg, int index, double &vx, double &vy){
            Ped::Tvector p = peds_sim_[index]->getPosition();
            vx = peds_sim_[index]->getVelocity().x;
            vy = peds_sim_[index]->getVelocity().y;
            double yaw= atan2(vy, vx);
            Point3d p_robot_pose_(p.x, p.y, yaw);
            p_msg.position.x = p.x;
            p_msg.position.y = p.y;
            p_msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
        };
};

#endif