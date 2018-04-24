#include <math.h>
#include "TaskUndock.h"
#include "floor_nav/TaskUndockConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_UNDOCK
#ifdef DEBUG_UNDOCK
#warning Debugging task UNDOCK
#endif


TaskIndicator TaskUndock::initialise() 
{
    ROS_INFO("Going to %.2f %.2f",cfg.goal_x);
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    x_init = tpose.x;
    y_init = tpose.y;
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskUndock::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init - tpose.y,x_init + cfg.goal_x-tpose.x);
    if (r < cfg.dist_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((y_init -tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,M_PI);
#ifdef DEBUG_UNDOCK
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
#endif
    if (fabs(alpha) > M_PI/9) {
        double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
#ifdef DEBUG_UNDOCK
        printf("Cmd v %.2f r %.2f\n",0.,rot);
#endif
        env->publishVelocity(0,rot);
    } else {
        double vel = cfg.k_v * r;
        if (vel < - cfg.max_velocity) vel = - cfg.max_velocity;
#ifdef DEBUG_UNDOCK
        printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
        env->publishVelocity(vel, 0);
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskUndock::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryUndock);
