#include "pedscene.h"
#include "rvoscene.h"
#include "ervoscene.h"
#include "emptyscene.h"
class SceneFactory{
    public:
        SceneFactory(){};
        boost::shared_ptr<Scene> makeAscene(string name, map<string, double> params_double, map<string, vector<double>> params_vecdouble){

            if(name ==  "pedscene"){
                /* code */
                return boost::make_shared<PedScene>(params_double, params_vecdouble);
            }
            else if (name == "rvoscene"){
                return boost::make_shared<RVOScene>(params_double, params_vecdouble);//new RVOScene(params);
            }
            else if (name == "ervoscene"){
                /* code */
                return boost::make_shared<ERVOScene>(params_double, params_vecdouble);//new ERVOScene(params);
            }
            else{
                return boost::make_shared<EmptyScene>(params_double, params_vecdouble);//new Scene(params);
            }
        }

};