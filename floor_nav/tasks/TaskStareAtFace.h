#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskStareAtFaceConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
            double target_angle;
        public:
            TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskStareAtFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
    {

        public:
            TaskFactoryStareAtFace(TaskEnvironmentPtr env) :
                Parent("TaskStareAtFace","turn heading to face the face (no joke !)",true,env) {}
            virtual ~TaskFactoryStareAtFace() {};
    };
};

#endif // TASK_SET_HEADING_H
