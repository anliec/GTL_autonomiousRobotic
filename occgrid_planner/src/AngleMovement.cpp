//
// Created by nsix on 3/23/18.
//

#include "AngleMovement.h"
#include <limits>

AngleMovement::AngleMovement(const int &base_angle, const int &deltaX, const int &deltaY, const int &deltaAngle, const float cost):
    base_angle(base_angle), dx(deltaX), dy(deltaY), da(deltaAngle), cost(cost)
{

}

AngleMovement::AngleMovement(): base_angle(-1), dx(-1), dy(-1), da(-1), cost(std::numeric_limits<float>::max())
{

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
    int delta_angle = angle_level - base_angle;
    if(delta_angle % NUMBER_OF_ANGLE_STEPS != 0){
        std::cerr << "[AngleMovement::get_rotate] Provided angle do not allow to compute a rotation" << std::endl;
        std::cerr << "\tbase angle: " << base_angle << std::endl;
        std::cerr << "\tangle level: " << angle_level << std::endl;
        return AngleMovement();
    }
    else{
        delta_angle /= NUMBER_OF_ANGLE_STEPS;
        delta_angle += 4;
        delta_angle %= 4;
        switch (delta_angle){
            case 0:
                return AngleMovement(base_angle, dx, dy, da, cost);
            case 1:
                return  AngleMovement(angle_level, -dy, dx, da, cost);
            case 2:
                return  AngleMovement(angle_level, -dx, -dy, da, cost);
            case 3:
                return  AngleMovement(angle_level, dy, -dx, da, cost);
            default:
                return AngleMovement(); //set to prevent warning, but cannot append tanks to %
        }
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

