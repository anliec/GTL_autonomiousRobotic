//
// Created by tboissin on 4/5/18.
//

#include <opencv-3.3.1/opencv2/opencv.hpp>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "TargetMapBuilder.h"

#define DEBUG

void addToHeap(GoalHeap &heap, const cv::Point &p, const float &point_score) {
    heap.push(std::pair<float, cv::Point>(point_score, p));
}

GoalHeap TargetMapBuilder::computeGoals(const cv::Mat_<uint8_t> &map, const cv::Point &robotLoc) {
    GoalHeap goals = GoalHeap();
    cv::MatSize mapShape = map.size;
    mapFrontierPoint_ = cv::Mat_<uint8_t>(mapShape[0], mapShape[1]);
    for (unsigned int j = 1; j < (mapShape[1]-1); j++) {
        for (unsigned int i = 1; i < (mapShape[0]-1); i++) {
            cv::Point p(i, j);
            // for all point of the map find free points
            if (map(p) == FREE) {
                // for each free point count unknown neighbors
                float unknown_count = 0.0f;
                for (int dj = -1; dj <= 1; dj++){
                    for (int di = -1; di <= 1; di++){
                        if (map(cv::Point(i + di, j + dj)) == UNKNOWN) {
                            unknown_count++;
                        }
                    }
                }
                // if there is more than 1 unknown neighbor this point is frontier point
                if (unknown_count > 0.0f){
                    // compute it's score and add it to the heap
                    float score = unknown_count / (powf(p.x-robotLoc.x, 2) + powf(p.x-robotLoc.x, 2) + 1.0f);
                    mapFrontierPoint_(p) = static_cast<uint8_t>(score*256);
                    addToHeap(goals, p, score);
                }
            }
        }
    }
#ifdef DEBUG
    ROS_INFO("Display map frontier");
    cv::imshow("OccGrid", mapFrontierPoint_);
#endif
    return goals;
}

TargetMapBuilder::TargetMapBuilder() {
    cv::namedWindow("OccGrid", CV_WINDOW_AUTOSIZE);
}
