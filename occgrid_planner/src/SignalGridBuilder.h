//
// Created by nsix on 4/12/18.
//

#ifndef PROJECT_SIGNALGRIDBUILDER_H
#define PROJECT_SIGNALGRIDBUILDER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <opencv2/opencv.hpp>



class SignalGridBuilder {
public:
    SignalGridBuilder();

    void signalHandler(const std_msgs::Float32 &msg);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber signalSub_;
    image_transport::Publisher mapPub_;
    tf::TransformListener listener_;
    cv::Mat_<uint8_t> sg_, sg_count_;
    cv::Point sg_center_;
    float sg_resolution_;
    float maxSignalValue;

    std::string frame_id_;
    std::string base_link_;
};


#endif //PROJECT_SIGNALGRIDBUILDER_H
