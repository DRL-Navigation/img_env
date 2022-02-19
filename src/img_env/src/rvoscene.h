#ifndef _rvoscene_
#define _rvoscene_

#include "scene.h"
#include <ervo_ros/RVO.h>
class RVOScene:public Scene{
    private: 
        RVO::RVOSimulator *rvo_sim;
        int ped_num_;

    public:

        RVOScene(map<string, double> params_double, map<string, vector<double>> params_vecdouble):Scene(params_double, params_vecdouble){
            rvo_sim = new RVO::RVOSimulator();
            rvo_sim->setTimeStep(step_hz_);
        };
        void setWayPoint(int index, vector<geometry_msgs::Point> traj, double px,  double py){};
        // virtual void step();
        void addObs(double pax, double pay, double pbx, double pby){
            std::vector<RVO::Vector2> obs_shape;
            obs_shape.push_back(RVO::Vector2(pax, pay));
            obs_shape.push_back(RVO::Vector2(pax, pby));
            obs_shape.push_back(RVO::Vector2(pbx, pby));
            obs_shape.push_back(RVO::Vector2(pbx, pay));
            rvo_sim->addObstacle(obs_shape);
        };

        void clearObs(){
            rvo_sim->clearObstacle();
        };

        void setPedPos(int index, double px,  double py){
            rvo_sim->setAgentPosition(index, RVO::Vector2(px, py));
        };

        void step(vector<RVO::Vector2> nextgoals,vector<RVO::Vector2> ps_, vector<float> rs_){
            for(int i=0; i<nextgoals.size(); i++){
                RVO::Vector2 goalVector = nextgoals[i] - rvo_sim->getAgentPosition(i);

                if (RVO::absSq(goalVector) > 1.0f) {
                goalVector = RVO::normalize(goalVector);
                }
                rvo_sim->setAgentPrefVelocity(i, goalVector);
            }
            rvo_sim -> doStep();
        }
        void setRobotPos(int index,double pax, double pay, double pbx, double pby,
                     double px,  double py,  double vx, double vy){
            rvo_sim->setAgentPosition(index+ped_num_, RVO::Vector2(px, py));
            rvo_sim->setAgentVelocity(index+ped_num_, RVO::Vector2(vx, vy));
        };

        void addPed(int ped_num, vector<double> ped_max_speed_){
            ped_num_ = ped_num;

            for (int i = 0; i < ped_num; i++)
                rvo_sim->addAgent(RVO::Vector2(0, 0), 0.5, 10, 5, 5, 0.5, ped_max_speed_[i]);
        };  

        void addRobot(int robo_num){
            for (int i = 0; i < robo_num; i++)
            if(relation_ped_robo==1){
                int k = rvo_sim->addAgent(RVO::Vector2(0, 0), 0.5, 10, 5, 5, 0.5, 0.6);
                cout << "Ped can see robot, add robot ok ! robot index " << k << endl;
                }
            };

            
        void processObs(){
            rvo_sim->processObstacles();
        }
        void getNewPosAndVel(geometry_msgs::Pose &p_msg, int index, double &vx, double &vy){
            RVO::Vector2 p = rvo_sim->getAgentPosition(index);
            RVO::Vector2 vel = rvo_sim->getAgentVelocity(index);
            vx = vel.x();
            vy = vel.y();
            double yaw= atan2(vy, vx);
            Point3d p_pose(p.x(), p.y(), yaw);
            p_msg.position.x = p.x();
            p_msg.position.y = p.y();
            p_msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
        };
};
#endif