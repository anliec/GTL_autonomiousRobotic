#ifndef TASK_GOTOPOSE_H
#define TASK_GOTOPOSE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToDockConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGoToDock : public TaskInstance<TaskGoToDockConfig,SimTasksEnv>
    {
        protected:
            double x_init,y_init, theta_init;
        public:
            TaskGoToDock(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoToDock() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoToDock : public TaskDefinition<TaskGoToDockConfig, SimTasksEnv, TaskGoToDock>
    {

        public:
            TaskFactoryGoToDock(TaskEnvironmentPtr env) : 
                Parent("GoToDock","Reach a desired destination and set heading",true,env) {}
            virtual ~TaskFactoryGoToDock() {};
    };
};

#endif // TASK_GOTOPOSE_H
