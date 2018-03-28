//
// Created by nsix on 3/23/18.
//

#include "AngleMovement.h"
#include <limits>

AngleMovement::AngleMovement(const int &base_angle, const int &deltaX, const int &deltaY, const int &deltaAngle, const float &costMalus):
    base_angle(base_angle), dx(deltaX), dy(deltaY), da(deltaAngle)
{
    cost = sqrtf(deltaX * deltaX + deltaY * deltaY) + costMalus;
}

AngleMovement::AngleMovement(): base_angle(-1), dx(-1), dy(-1), da(-1), cost(std::numeric_limits<float>::max())
{

}

AngleMovement::AngleMovement(const AngleMovement &o)
{
    this->base_angle = o.base_angle;
    this->da = o.da;
    this->cost = o.cost;
    this->dx = o.dx;
    this->dy = o.dy;
}

//AngleMovement AngleMovement::get_sim() const
//{
//    return AngleMovement(base_angle, dx, -dy, -da, cost);
//}

/**
 * rotate the current move to go to the given angle plan index
 * @param angle_level index of the destination plan
 * @return a new AngleMovement object
 */
AngleMovement AngleMovement::get_rotate(const unsigned &angle_level) const
{
    AngleMovement ret = AngleMovement(*this);
    int err = ret.rotate(angle_level);
    if(err == 0){
        return ret;
    } else{
        std::cerr << "[AngleMovement::get_rotate] error when computing rotation" << std::endl;
        return AngleMovement();
    }
}

Move3D AngleMovement::get_movement() const
{
    Move3D ret;
    ret.angle = da;
    ret.pt = cv::Point(dx, dy);
    return ret;
}

int AngleMovement::get_dx() const{
    return dx;
}

int AngleMovement::get_dy() const{
    return dy;
}

int AngleMovement::get_da() const{
    return da;
}

float AngleMovement::get_cost() const{
    return cost;
}

int AngleMovement::rotate(const unsigned &angle_level)
{
    int delta_angle = angle_level - base_angle;
    if(delta_angle % NUMBER_OF_ANGLE_STEPS != 0){
        std::cerr << "[AngleMovement::rotate] Provided angle do not allow to compute a rotation" << std::endl;
        std::cerr << "\tbase angle: " << base_angle << std::endl;
        std::cerr << "\tangle level: " << angle_level << std::endl;
        return -1;
    }
    else{
        delta_angle /= NUMBER_OF_ANGLE_STEPS;
        delta_angle += 4;
        delta_angle %= 4;
        int tmp;
        base_angle = angle_level;
        switch (delta_angle){
            case 0:
                return 0;
            case 1:
                tmp = dx;
                dx = -dy;
                dy = tmp;
                return 0;
            case 2:
                dx = -dx;
                dy = -dy;
                return 0;
            case 3:
                tmp = dx;
                dx = dy;
                dy = -tmp;
                return 0;
            default:
                std::cerr << "[AngleMovement::rotate] Provided angle do not allow to compute a rotation" << std::endl;
                std::cerr << "\tbase angle: " << base_angle << std::endl;
                std::cerr << "\tangle level: " << angle_level << std::endl;
                std::cerr << "\tdelta angle: " << delta_angle << std::endl;
                return -1; //set to prevent warning, but cannot append tanks to %
        }
    }
}

