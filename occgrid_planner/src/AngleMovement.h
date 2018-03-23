//
// Created by nsix on 3/23/18.
//
#ifndef PROJECT_ANGLEMOVEMENT_H
#define PROJECT_ANGLEMOVEMENT_H

#include <opencv2/opencv.hpp>

const static unsigned NUMBER_OF_ANGLES_LEVELS = 8;
const static unsigned NUMBER_OF_ANGLE_STEPS = NUMBER_OF_ANGLES_LEVELS / 4;


struct Move3D{
    cv::Point pt;
    int angle;
};

class AngleMovement {

public:
    AngleMovement(const int &base_angle, const int &deltaX, const int &deltaY, const int &deltaAngle, const float cost);
    AngleMovement();

//    AngleMovement get_sim() const;
    AngleMovement get_rotate(const unsigned &angle_level) const;

    Move3D get_movement() const;

    int get_dx() const;
    int get_dy() const;
    int get_da() const;
    float get_cost() const;

private:
    int dx, dy, da;
    int base_angle;
    float cost;
};


#endif //PROJECT_ANGLEMOVEMENT_H
