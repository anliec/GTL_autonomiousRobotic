//
// Created by nsix on 4/12/18.
//

#include "SignalGridBuilder.h"
#include <opencv2/highgui/highgui.hpp>

const static int UNKNOWN_SIGNAL = 0;

SignalGridBuilder::SignalGridBuilder()
{
    nh_ = ros::NodeHandle("~");

    int sizeX, sizeY;

    nh_.param("base_frame", base_link_, std::string("/body"));
    nh_.param("sg_grid", frame_id_, std::string("/map"));
    nh_.param("mapSizeX", sizeX, 256);
    nh_.param("mapSizeX", sizeY, 256);
    nh_.param("resolution", sg_resolution_, 1.0f);
//    nh_.param("maxSignalValue", maxSignalValue, 1.0f);

    sg_ = cv::Mat_<uint8_t>(sizeY, sizeX);
    sg_count_ = cv::Mat_<uint8_t>(sizeY, sizeX);
    sg_center_ = cv::Point(sizeY / 2, sizeX / 2);

    explored = cv::Rect(sg_center_.x-1, sg_center_.y-1, 3, 3);
    maxSignalValue = 0.1f;

    for (int x = 0; x < sizeX; ++x) {
        for (int y = 0; y < sizeY; ++y) {
            sg_(y,x) = UNKNOWN_SIGNAL;
            sg_count_(y,x) = 0;
        }
    }

    //subscribe to the signal
    signalSub_ = nh_.subscribe("/SignalGridBuilder/signal", 1, &SignalGridBuilder::signalHandler, this);
    //advertise map
    image_transport::ImageTransport it(nh_);
    mapPub_ = it.advertise("/SignalGridBuilder/signalMap", 1, false);
}


void SignalGridBuilder::signalHandler(const std_msgs::Float32 &msg)
{
    //get current position
    tf::StampedTransform transform;
    listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
    cv::Point curPoint(int(transform.getOrigin().x() / sg_resolution_),
                       int(transform.getOrigin().y() / sg_resolution_));
    curPoint += sg_center_;

    //update max values if needed
    if(maxSignalValue < fabsf(msg.data)){
        float factor = 254.0f * fabsf(msg.data) / maxSignalValue;
        maxSignalValue = fabsf(msg.data);
        for (int x = 0; x < sg_.size[1]; ++x) {
            for (int y = 0; y < sg_.size[0]; ++y) {
                if(sg_(y,x) != UNKNOWN_SIGNAL){
                    sg_(y,x) = static_cast<uint8_t>(fabsf(factor * float(sg_(y,x))));
                }
            }
        }
    }

    //set current position to the given value
    if(sg_count_(curPoint) == 0){
        sg_(curPoint) = static_cast<uint8_t >(254.0 * fabsf(msg.data) / maxSignalValue);
    }
    else{
        float local_value = 254.0f * fabsf(msg.data) / maxSignalValue;
        float new_value = (sg_(curPoint) * sg_count_(curPoint) + local_value) / (sg_count_(curPoint) + 1);
        sg_(curPoint) = static_cast<uint8_t >(new_value);
    }
    sg_count_(curPoint) += 1;

    //update explored rect
    if(explored.x > curPoint.x){
        explored.width += explored.x - curPoint.x;
        explored.x = curPoint.x;
    }
    else if(explored.width + explored.x < curPoint.x){
        explored.width = curPoint.x - explored.x;
    }
    if(explored.y > curPoint.y){
        explored.height += explored.y - curPoint.y;
        explored.y = curPoint.y;
    }
    else if(explored.height + explored.y < curPoint.y){
        explored.height = curPoint.y - explored.y;
    }

    if(mapPub_.getNumSubscribers() != 0){
        cv::Mat_<uint8_t> zoom = sg_(explored);
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", zoom).toImageMsg();
        mapPub_.publish(imageMessage);
    }
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "signal_grid_builder");
    SignalGridBuilder sgb;

    while (ros::ok()) {
        ros::spinOnce();
        cv::waitKey(50);
    }
}
