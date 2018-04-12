//
// Created by nsix on 4/12/18.
//

#include "SignalGridBuilder.h"
#include <opencv2/highgui/highgui.hpp>


SignalGridBuilder::SignalGridBuilder()
{
    nh_ = ros::NodeHandle("~");

    int sizeX, sizeY;

    nh_.param("base_frame", base_link_, std::string("/body"));
    nh_.param("sg_grid", frame_id_, std::string("/map"));
    nh_.param("mapSizeX", sizeX, 256);
    nh_.param("mapSizeX", sizeY, 256);
    nh_.param("resolution", sg_resolution_, 1.0f);
    nh_.param("maxSignalValue", maxSignalValue, 1.0f);

    sg_ = cv::Mat_<uint8_t>(sizeY, sizeX);
    sg_count_ = cv::Mat_<uint8_t>(sizeY, sizeX);
    sg_center_ = cv::Point(sizeY / 2, sizeX / 2);

    for (int x = 0; x < sizeX; ++x) {
        for (int y = 0; y < sizeY; ++y) {
            sg_(y,x) = 255;
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

    //set current position to the given value
    sg_(curPoint) = static_cast<uint8_t >(msg.data / maxSignalValue);

    if(mapPub_.getNumSubscribers() != 0){
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", sg_).toImageMsg();
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
