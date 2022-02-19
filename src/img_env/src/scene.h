#ifndef   _scene_   
#define   _scene_
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
#include <ervo_ros/RVO.h>
class Scene{
   
    public:
        bool relation_ped_robo;
        vector<double> ped_max_speed;
        double step_hz_;
        Scene(){};
        Scene(map<string, double> params_double, map<string, vector<double>> params_vecdouble){
            relation_ped_robo = params_double["relation_ped_robo"];
            step_hz_ = params_double["step_hz_"];
        };
        virtual void step(vector<RVO::Vector2> nextgoals,vector<RVO::Vector2> ps_, vector<float> rs_) = 0;
        virtual void addObs(double pax, double pay, double pbx, double pby) = 0;
        virtual void clearObs() = 0;
        virtual void setPedPos(int index,double x,double y) = 0;
        virtual void setRobotPos(int index,double pax, double pay, double pbx, double pby, double px,  double py, double vx, double vy) = 0;
        virtual void setWayPoint(int index, vector<geometry_msgs::Point> traj, double px,  double py) = 0;
        virtual void processObs() = 0;
        virtual void addPed(int ped_num, vector<double> ped_max_speed_) = 0;
        virtual void addRobot(int robo_num) = 0;
        virtual void getNewPosAndVel(geometry_msgs::Pose &p_msg, int index, double &vx, double &vy) = 0;
};

#endif