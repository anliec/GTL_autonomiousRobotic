//
// Created by nsix on 4/12/18.
//

#include "WifiSignalGridBuilder.h"
#include <opencv2/highgui/highgui.hpp>

const static int UNKNOWN_SIGNAL = 0;

WifiSignalGridBuilder::WifiSignalGridBuilder()
{
    nh_ = ros::NodeHandle("~");

    int sizeX, sizeY;

    nh_.param("base_frame", base_link_, std::string("/body"));
    nh_.param("sg_grid", frame_id_, std::string("/map"));
    nh_.param("mapSizeX", sizeX, 256);
    nh_.param("mapSizeX", sizeY, 256);
    nh_.param("resolution", sg_resolution_, 1.0f);

    sg_center_ = cv::Point(sizeY / 2, sizeX / 2);

    explored = cv::Rect(sg_center_.x-1, sg_center_.y-1, 3, 3);
    maxSignalValue = 0.1f;

    //subscribe to the signal
    signalSub_ = nh_.subscribe("/WifiSignalGridBuilder/signal", 1, &WifiSignalGridBuilder::signalHandler, this);
    //advertise map
    imageTransport = new image_transport::ImageTransport(nh_);
}

WifiSignalGridBuilder::~WifiSignalGridBuilder() {
    delete imageTransport;
}

void WifiSignalGridBuilder::signalHandler(const occgrid_planner::wifi &msg)
{
    unsigned wifiId;
    try {
        wifiId = wifiNameToId.at(msg.MAC);
    }
    catch (std::out_of_range &e){
        wifiId = addEmptyMap(msg.MAC);
    }


    //get current position
    tf::StampedTransform transform;
    listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
    cv::Point curPoint(int(transform.getOrigin().x() / sg_resolution_),
                       int(transform.getOrigin().y() / sg_resolution_));
    curPoint += sg_center_;

    //update max values if needed
    if(maxSignalValue < fabsf(msg.dB)){
        float factor = 254.0f * fabsf(msg.dB) / maxSignalValue;
        maxSignalValue = fabsf(msg.dB);
        for (int x = 0; x < sg_[wifiId].size[1]; ++x) {
            for (int y = 0; y < sg_[wifiId].size[0]; ++y) {
                if(sg_[wifiId](y,x) != UNKNOWN_SIGNAL){
                    sg_[wifiId](y,x) = static_cast<uint8_t>(fabsf(factor * float(sg_[wifiId](y,x))));
                }
            }
        }
    }

    //set current position to the given value
    if(sg_count_[wifiId](curPoint) == 0){
        sg_[wifiId](curPoint) = static_cast<uint8_t >(254.0 * fabsf(msg.dB) / maxSignalValue);
    }
    else{
        float local_value = 254.0f * fabsf(msg.dB) / maxSignalValue;
        float new_value = (sg_[wifiId](curPoint) * sg_count_[wifiId](curPoint) + local_value) / (sg_count_[wifiId](curPoint) + 1);
        sg_[wifiId](curPoint) = static_cast<uint8_t >(new_value);
    }
    sg_count_[wifiId](curPoint) += 1;

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

    if(mapPub_[wifiId].getNumSubscribers() != 0){
        cv::Mat_<uint8_t> zoom = sg_[wifiId](explored);
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", zoom).toImageMsg();
        mapPub_[wifiId].publish(imageMessage);
    }
}

unsigned WifiSignalGridBuilder::addEmptyMap(std::string name) {
    auto newId = static_cast<unsigned>(wifiNameToId.size());
    wifiNameToId.insert(std::pair<std::string, unsigned>(name, newId));

    cv::Mat_<uint8_t> sg(mapSizeY, mapSizeX);
    cv::Mat_<uint8_t> sg_count(mapSizeY, mapSizeX);

    for (unsigned x = 0; x < mapSizeX; ++x) {
        for (unsigned y = 0; y < mapSizeY; ++y) {
            sg(y,x) = UNKNOWN_SIGNAL;
            sg_count(y,x) = 0;
        }
    }

    sg_.push_back(sg);
    sg_count_.push_back(sg_count);

    mapPub_.push_back(imageTransport->advertise("/WifiSignalGridBuilder/signalMap" + std::to_string(newId), 1, false));

    return newId;
}




int main(int argc, char *argv[]) {
    ros::init(argc, argv, "signal_grid_builder");
    WifiSignalGridBuilder sgb;

    while (ros::ok()) {
        ros::spinOnce();
        cv::waitKey(50);
    }
}
