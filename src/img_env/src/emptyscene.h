#ifndef _emptyscene_
#define _emptyscene_

#include "scene.h"
class EmptyScene:public Scene{
    public:

        EmptyScene(map<string, double> params_double, map<string, vector<double>> params_vecdouble):Scene(params_double, params_vecdouble){};

        void setWayPoint(int index, vector<geometry_msgs::Point> traj, double px,  double py){};

        void addObs(double pax, double pay, double pbx, double pby){};

        void clearObs(){};

        void setPedPos(int index, double px,  double py){};

        void step(vector<RVO::Vector2> nextgoals,vector<RVO::Vector2> ps_, vector<float> rs_){};

        void setRobotPos(int index,double pax, double pay, double pbx, double pby,
                     double px,  double py,  double vx, double vy){};

        void addPed(int ped_num, vector<double> ped_max_speed_){};

        void addRobot(int robo_num){};

        void processObs(){};

        void getNewPosAndVel(geometry_msgs::Pose &p_msg, int index, double &vx, double &vy){};
};
#endif