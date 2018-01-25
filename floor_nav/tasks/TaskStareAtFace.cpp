#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    initial_heading = tpose.theta;

    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    // compute the heading from the face ROI position
    target_angle = (env->getFacePosition()*2./cfg.camera_width)*30.;
    ROS_INFO("Setting heading to %.2f deg", target_angle);
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double alpha = remainder(initial_heading+target_angle-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_theta*alpha;
    if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
