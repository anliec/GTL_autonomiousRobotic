#ifndef TASK_GoToBase_H
#define TASK_GoToBase_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToBaseConfig.h"
#include <actionlib/client/simple_action_client.h>
#include "kobuki_msgs/AutoDockingAction.h"

typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> Client;

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGoToBase : public TaskInstance<TaskGoToBaseConfig,SimTasksEnv>
    {
    public:
        TaskGoToBase(TaskDefinitionPtr def, TaskEnvironmentPtr env);
        virtual ~TaskGoToBase();

        virtual TaskIndicator initialise();

        virtual TaskIndicator iterate();

        virtual TaskIndicator terminate();

    private:
        Client *client;
        actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;
    };
    class TaskFactoryGoToBase : public TaskDefinition<TaskGoToBaseConfig, SimTasksEnv, TaskGoToBase>
    {

        public:
            TaskFactoryGoToBase(TaskEnvironmentPtr env) : 
                Parent("GoToBase","Go To Base aimlessly forever",true,env) {}
            virtual ~TaskFactoryGoToBase() {};
    };
};

#endif // TASK_GoToBase_H
