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

#include <wifi_publisher/wifi.h>



class WifiSignalGridBuilder {
public:
    WifiSignalGridBuilder();
    ~WifiSignalGridBuilder();

    void signalHandler(const wifi_publisher::wifi &msg);

protected:
    unsigned addEmptyMap(std::string mac, std::string ssid);

    ros::NodeHandle nh_;
    ros::Subscriber signalSub_;
    std::vector<image_transport::Publisher> mapPub_;
    tf::TransformListener listener_;
    std::vector<cv::Mat_<uint8_t>> sg_, sg_count_;
    std::map<std::string, unsigned> wifiNameToId_;
    std::vector<std::string> idToSSID_;
    cv::Point sg_center_;
    float sg_resolution_;
    float maxSignalValue_;

    int mapSizeX_, mapSizeY_;

    cv::Rect explored_;

    std::string frame_id_;
    std::string base_link_;

    image_transport::ImageTransport *imageTransport_;
};


#endif //PROJECT_SIGNALGRIDBUILDER_H
