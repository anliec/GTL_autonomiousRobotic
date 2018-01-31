#ifndef TASK_GOTOPOSE_H
#define TASK_GOTOPOSE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGotoPoseConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGotoPose : public TaskInstance<TaskGotoPoseConfig,SimTasksEnv>
    {
        protected:
            double x_init,y_init;
        public:
            TaskGotoPose(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGotoPose() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGotoPose : public TaskDefinition<TaskGotoPoseConfig, SimTasksEnv, TaskGotoPose>
    {

        public:
            TaskFactoryGotoPose(TaskEnvironmentPtr env) : 
                Parent("GotoPose","Reach a desired destination and set heading",true,env) {}
            virtual ~TaskFactoryGotoPose() {};
    };
};

#endif // TASK_GOTOPOSE_H
