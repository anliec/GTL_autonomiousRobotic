#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

#ifdef TURTLEBOT
    const static int CAMERA_ANGLE = 57; //degree value
    const static int CAMERA_WIDTH = 1280;
//#endif
//
//#ifdef VREP
#else
    const static int CAMERA_ANGLE = 45; //degree value
    const static int CAMERA_WIDTH = 256;
#endif

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    initial_heading = tpose.theta;

    // compute the heading from the face ROI position
    double target_position_rel = env->getFacePosition() - CAMERA_WIDTH / 2;
    target_angle = -target_position_rel * CAMERA_ANGLE / 180 * 3.1416 / CAMERA_WIDTH;

    ROS_INFO("Position to target on camera: %d (relative: %.2f)", env->getFacePosition(), target_position_rel);
    ROS_INFO("Rotation angle to reach target: %.2f rad = %.2f deg", target_angle, target_angle * 180 / 3.1416);
    ROS_INFO("initial_heading: %.2f", initial_heading);

    ROS_INFO("M_PI = %f", M_PI);

//    target_angle += initial_heading;

    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    ROS_INFO("Setting heading to %.2f", target_angle + initial_heading);
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    ROS_INFO("current_heading: %.2f", tpose.theta);
    double alpha = remainder(initial_heading + target_angle - tpose.theta, 2*M_PI);
    if (fabs(alpha) < cfg.angle_threshold)
    {
		return TaskStatus::TASK_COMPLETED;
    }
    else
    {
        double rot = cfg.k_theta * alpha;
        if (rot > cfg.max_angular_velocity)  rot = cfg.max_angular_velocity;
        if (rot < -cfg.max_angular_velocity) rot = -cfg.max_angular_velocity;
        env->publishVelocity(0.0, rot);
        return TaskStatus::TASK_RUNNING;
    }
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
