#ifndef TASK_UNDOCK_H
#define TASK_UNDOCK_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskUndockConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskUndock : public TaskInstance<TaskUndockConfig,SimTasksEnv>
    {
        protected:
            double x_init,y_init;
        public:
            TaskUndock(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskUndock() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryUndock : public TaskDefinition<TaskUndockConfig, SimTasksEnv, TaskUndock>
    {

        public:
            TaskFactoryUndock(TaskEnvironmentPtr env) : 
                Parent("Undock","Reach a desired destination",true,env) {}
            virtual ~TaskFactoryUndock() {};
    };
};

#endif // TASK_UNDOCK_H
