//
// Created by nsix on 4/12/18.
//

#include "WifiSignalGridBuilder.h"
#include <opencv2/highgui/highgui.hpp>

const static int UNKNOWN_SIGNAL = 0;

WifiSignalGridBuilder::WifiSignalGridBuilder()
{
    nh_ = ros::NodeHandle("~");

    nh_.param("base_frame", base_link_, std::string("/body"));
    nh_.param("sg_grid", frame_id_, std::string("/map"));
    nh_.param("mapSizeX_", mapSizeX_, 256);
    nh_.param("mapSizeX_", mapSizeY_, 256);
    nh_.param("resolution", sg_resolution_, 1.0f);

    sg_center_ = cv::Point(mapSizeY_ / 2, mapSizeX_ / 2);

    explored_ = cv::Rect(sg_center_.x-1, sg_center_.y-1, 3, 3);
    maxSignalValue_ = 0.1f;

    //subscribe to the signal
    signalSub_ = nh_.subscribe("/WifiSignalGridBuilder/signal", 1, &WifiSignalGridBuilder::signalHandler, this);
    //advertise map
    imageTransport_ = new image_transport::ImageTransport(nh_);
}

WifiSignalGridBuilder::~WifiSignalGridBuilder() {
    delete imageTransport_;
    for(const auto &pair : wifiNameToId_)
    {
        const unsigned &i = pair.second;
        std::string path = "~/map_" + idToSSID_[i] + "_" + pair.first + ".png";
        cv::imwrite(path, sg_[i]);
    }
}

void WifiSignalGridBuilder::signalHandler(const wifi_publisher::wifi &msg)
{
    unsigned wifiId;
    try {
        wifiId = wifiNameToId_.at(msg.MAC);
    }
    catch (std::out_of_range &e){
        wifiId = addEmptyMap(msg.MAC, msg.ssid);
    }


    //get current position
    tf::StampedTransform transform;
    listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
    cv::Point curPoint(int(transform.getOrigin().x() / sg_resolution_),
                       int(transform.getOrigin().y() / sg_resolution_));
    curPoint += sg_center_;

    //update max values if needed
    if(maxSignalValue_ < fabsf(msg.dB)){
        float factor = 254.0f * fabsf(msg.dB) / maxSignalValue_;
        maxSignalValue_ = fabsf(msg.dB);
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
        sg_[wifiId](curPoint) = static_cast<uint8_t >(254.0 * fabsf(msg.dB) / maxSignalValue_);
    }
    else{
        float local_value = 254.0f * fabsf(msg.dB) / maxSignalValue_;
        float new_value = (sg_[wifiId](curPoint) * sg_count_[wifiId](curPoint) + local_value) / (sg_count_[wifiId](curPoint) + 1);
        sg_[wifiId](curPoint) = static_cast<uint8_t >(new_value);
    }
    sg_count_[wifiId](curPoint) += 1;

    //update explored_ rect
    if(explored_.x > curPoint.x){
        explored_.width += explored_.x - curPoint.x;
        explored_.x = curPoint.x;
    }
    else if(explored_.width + explored_.x < curPoint.x){
        explored_.width = curPoint.x - explored_.x;
    }
    if(explored_.y > curPoint.y){
        explored_.height += explored_.y - curPoint.y;
        explored_.y = curPoint.y;
    }
    else if(explored_.height + explored_.y < curPoint.y){
        explored_.height = curPoint.y - explored_.y;
    }

    if(mapPub_[wifiId].getNumSubscribers() != 0){
        cv::Mat_<uint8_t> zoom = sg_[wifiId](explored_);
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", zoom).toImageMsg();
        mapPub_[wifiId].publish(imageMessage);
    }
}

unsigned WifiSignalGridBuilder::addEmptyMap(std::string mac, std::string ssid) {
    auto newId = static_cast<unsigned>(wifiNameToId_.size());
    wifiNameToId_.insert(std::pair<std::string, unsigned>(mac, newId));

    cv::Mat_<uint8_t> sg(mapSizeY_, mapSizeX_);
    cv::Mat_<uint8_t> sg_count(mapSizeY_, mapSizeX_);

    for (int x = 0; x < mapSizeX_; ++x) {
        for (int y = 0; y < mapSizeY_; ++y) {
            sg(y,x) = UNKNOWN_SIGNAL;
            sg_count(y,x) = 0;
        }
    }

    sg_.push_back(sg);
    sg_count_.push_back(sg_count);

    idToSSID_.push_back(ssid);

    mapPub_.push_back(imageTransport_->advertise("/WifiSignalGridBuilder/signalMap" + std::to_string(newId), 1, false));

    return newId;
}




int main(int argc, char *argv[]) {
    ros::init(argc, argv, "wifi_signal_grid_builder");
    WifiSignalGridBuilder sgb;

    while (ros::ok()) {
        ros::spinOnce();
        cv::waitKey(50);
    }
}
