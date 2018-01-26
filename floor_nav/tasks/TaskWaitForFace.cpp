#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"

using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{
    ros::Time currentTime, lastFaceTime;
    ros::Duration deltaTime;

    currentTime = ros::Time::now();
    lastFaceTime = env->getLastFaceTime();
    deltaTime = currentTime - lastFaceTime;

    ROS_DEBUG("delta Time from last seen face is %f", deltaTime.toSec());

    if (deltaTime.sec == 0) {
        ROS_INFO("Detected Face %.2f seconds ago",deltaTime.toSec());
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);
