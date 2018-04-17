//
// Created by tboissin on 4/5/18.
//

#include <opencv-3.3.1/opencv2/opencv.hpp>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "TargetMapBuilder.h"

//#define DEBUG

const static float MAX_NEIGHBORS_DIST = 5;

void addToHeap(GoalHeap &heap, const cv::Point &p, const float &point_score) {
    heap.push(std::pair<float, cv::Point>(point_score, p));
}

GoalHeap TargetMapBuilder::computeGoals(const cv::Mat_<uint8_t> &map, const cv::Point &robotLoc, const float &robotHeading) {
    GoalHeap goals = GoalHeap();
    cv::MatSize mapShape = map.size;
    mapFrontierPoint_ = cv::Mat_<uint8_t>(mapShape[0], mapShape[1]);
    std::vector<HeapElement> frontierPoints;
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
                            unknown_count += 1.0f;
                        }
                    }
                }
                // if there is more than 1 unknown neighbor this point is frontier point
                if (unknown_count > 0.0f){
                    // compute it's baseScore and add it to the heap if far enough from the robot
                    float distSquared = powf(p.x-robotLoc.x, 2) + powf(p.x-robotLoc.x, 2);
                    float baseScore = unknown_count * distSquared * expf(distSquared * -0.01f); // 0.04 = 1 / 10^2
                    frontierPoints.emplace_back(baseScore, p);
                    mapFrontierPoint_(p) = static_cast<uint8_t>(baseScore*0.256f);
                }
            }
        }
    }
    // add the score of neighbors to the base score computed above
    // can be done in n*log(n) but...
    for(HeapElement &p1 : frontierPoints){
        float score = 0.0f;
        for(HeapElement &p2 : frontierPoints){
            //if the points are close enough add their score
            if((abs(p1.second.x-p2.second.x) + abs(p1.second.y-p2.second.y)) < MAX_NEIGHBORS_DIST){
                score += p2.first;
            }
        }
        //multiply computed score by the an factor depending on angle with robot
        //factor is: pi^2 - deltaAngle^2 + 1
        //with deltaAngle the angle between the robot and the current point in [-pi; pi]
//        cv::Point move_vector = p1.second - robotLoc;
//        float angle = atan2f(move_vector.y, move_vector.x);
//        float deltaAngle = fmodf(angle - robotHeading + 4.0f * float(M_PI), float(2.0 * M_PI));
//        deltaAngle = deltaAngle - float(M_PI);
//        score *= (M_PI * M_PI) - (deltaAngle * deltaAngle) + 1;
        addToHeap(goals, p1.second, score);
    }

#ifdef DEBUG
    ROS_INFO("Display map frontier");
    cv::imshow("OccGrid", mapFrontierPoint_);
#endif
    return goals;
}

TargetMapBuilder::TargetMapBuilder() {
#ifdef DEBUG
    cv::namedWindow("OccGrid", CV_WINDOW_AUTOSIZE);
#endif
}
