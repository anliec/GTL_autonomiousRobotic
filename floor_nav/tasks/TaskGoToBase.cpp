#include "TaskGoToBase.h"
#include "floor_nav/TaskGoToBaseConfig.h"
#include "kobuki_msgs/AutoDockingGoal.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GoToBase
#ifdef DEBUG_GoToBase
    #warning Debugging task GoToBase
#endif

TaskGoToBase::TaskGoToBase(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {

}


TaskGoToBase::~TaskGoToBase() {
    if(client != nullptr){
        delete client;
    }
}


TaskIndicator TaskGoToBase::initialise() 
{
    ROS_INFO("Initialising Docking Task");
    client = new Client("dock_drive_action", true);
    client->waitForServer();
    kobuki_msgs::AutoDockingGoal goal;
    client->sendGoal(goal);
//    ros::Time startTime = ros::Time::now();
    ROS_INFO("Initialisation of Docking Task completed !");
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToBase::iterate()
{
    dock_state = client->getState();
    ROS_INFO("Docking status: %s",dock_state.toString().c_str());
    if (dock_state == actionlib::SimpleClientGoalState::SUCCEEDED){
        printf("Docked !");
        return TaskStatus::TASK_COMPLETED;
    }
//    else if(cfg.timeout > 0 && cfg.timeout < ros::Time::now().toSec() - startTime.toSec()) {
//        return TaskStatus::TASK_FAILED;
//    }
	return TaskStatus::TASK_RUNNING;
}


TaskIndicator TaskGoToBase::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToBase);
