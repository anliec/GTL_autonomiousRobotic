//
// Created by tboissin on 4/5/18.
//

#ifndef PROJECT_TARGETMAPBUILDER_H
#define PROJECT_TARGETMAPBUILDER_H

#define UNKNOWN 0xFF
#define FREE 0xFF
#define OCCUPIED 0x00

#include <opencv-3.3.1/opencv2/core/types.hpp>
#include <queue>
#include <opencv2/core/core.hpp>

typedef std::pair<float, cv::Point> HeapElement;

struct HeapElementCompare{
    bool operator()(HeapElement const &left, HeapElement const &right){
        return left.first > right.first;
    }
};

typedef std::priority_queue<HeapElement, std::vector<HeapElement>, HeapElementCompare> GoalHeap;

static void addToHeap(GoalHeap &heap, const cv::Point &p, const float &point_score);


class TargetMapBuilder {

protected:
    GoalHeap goals;

    cv::Mat_<uint8_t> map_, mapFrontierPoint_;

public:
    TargetMapBuilder();

    /**
     * compute the head containing the goals to explore
     * @param map : map with inflated obstacles
     * @param robotLoc : position of the robot respecting to the map
     * @return : the heap containing the goals to explore
     */
    GoalHeap computeGoals(const cv::Mat_<uint8_t> &map, const cv::Point &robotLoc);
};


#endif //PROJECT_TARGETMAPBUILDER_H
