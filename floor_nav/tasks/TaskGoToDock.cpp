#include <math.h>
#include "TaskGoToDock.h"
#include "floor_nav/TaskGoToDockConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTOPOSE
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToDock::initialise() 
{
    ROS_INFO("Going to %.2f %.2f %.2f",cfg.goal_x,cfg.goal_y,cfg.goal_theta);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToDock::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
    // theta is defined as the angular difference between current heading and goal angle
    double theta = remainder(tpose.theta - (cfg.goal_theta + theta_init) ,2*M_PI);
    if ((r < cfg.dist_threshold) && (theta < cfg.angular_threshold)){
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
#ifdef DEBUG_GOTOPOSE
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
#endif
    if (cfg.smart) {// smart method
        if (r > cfg.dist_threshold) {// step 1
            // TODO: check what happends if tposx-goal_X < 0 (seems ok according to doc)
            double rot = (alpha/M_PI) * cfg.max_angular_velocity;
            double vel = (cfg.max_velocity*hypot(tpose.x-cfg.goal_x, tpose.x-cfg.goal_x));
            if (vel > cfg.max_velocity) vel = cfg.max_velocity;
            vel = vel*exp(-pow(rot, 2)/cfg.sigma2);/*pow usage need to be checked*/
#ifdef DEBUG_GOTOPOSE
            printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
            env->publishVelocity(vel, rot);
        } else {
            double rot = ((theta > 0) ? +1 : -1) * cfg.max_angular_velocity;
#ifdef DEBUG_GOTOPOSE
            printf("Cmd v % .2f r %.2f\n",0.,rot);
#endif
            env->publishVelocity(0, rot);
        }
    } else {// dumb method
        if ((fabs(alpha) > M_PI / 9) && (r > cfg.dist_threshold)) {// step 1
            double rot = (alpha/M_PI) * cfg.max_angular_velocity;
#ifdef DEBUG_GOTOPOSE
            printf("Cmd v % .2f r %.2f\n",0.,rot);
#endif
            env->publishVelocity(0, rot);
        } else if (r > cfg.dist_threshold) {//step 2
            double vel = cfg.k_v * r;
            double rot = (alpha/M_PI) * cfg.max_angular_velocity;
            if (vel > cfg.max_velocity) vel = cfg.max_velocity;
#ifdef DEBUG_GOTOPOSE
            printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
            env->publishVelocity(vel, rot);
        } else {//step 3
            double rot = ((theta > 0) ? +1 : -1) * cfg.max_angular_velocity;
#ifdef DEBUG_GOTOPOSE
            printf("Cmd v % .2f r %.2f\n",0.,rot);
#endif
            env->publishVelocity(0, rot);
        }
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToDock::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToDock);
