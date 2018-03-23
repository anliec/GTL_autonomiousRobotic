//
// Created by nsix on 3/23/18.
//

#include "MovementGenerator.h"

MovementGenerator::MovementGenerator()
{
    //TODO: set real cost (to ensure A* accuracy)
    //set possible movement for angle = 0 / 90 / 180 / 270
    possibleMove[0].push_back(AngleMovement(0, 0,  0,  1, 1.0f)); // rotate on the spot
    possibleMove[0].push_back(AngleMovement(0, 1, -1,  2, 1.5f)); // diagonal + 90 degree
    possibleMove[0].push_back(AngleMovement(0, 2, -1,  1, 1.5f)); // diagonal + 1x + 45 degree
    //symmetrical
    possibleMove[0].push_back(AngleMovement(0, 0,  0, -1, 1.0f)); // rotate on the spot
    possibleMove[0].push_back(AngleMovement(0, 1,  1, -2, 1.5f)); // diagonal + 90 degree
    possibleMove[0].push_back(AngleMovement(0, 2,  1, -1, 1.5f)); // diagonal + 1x + 45 degree
    //add forward movement
    possibleMove[0].push_back(AngleMovement(0, 1,  0,  0, 1.0f));

    //set possible movement for angle = 45 / 135 / 225 / 315
    possibleMove[1].push_back(AngleMovement(0, 0,  0,  1, 1.0f)); // rotate on the spot
    possibleMove[1].push_back(AngleMovement(0, 1,  0, -2, 1.5f)); // diagonal + 90 degree
    possibleMove[1].push_back(AngleMovement(0, 2, -1, -1, 1.5f)); // diagonal + 1x + 45 degree
    //symmetrical
    possibleMove[1].push_back(AngleMovement(0, 0,  0, -1, 1.0f)); // rotate on the spot
    possibleMove[1].push_back(AngleMovement(0, 0, -1,  2, 1.5f)); // diagonal + 90 degree
    possibleMove[1].push_back(AngleMovement(0, 1, -2,  1, 1.5f)); // diagonal + 1y + 45 degree
    //add forward movement
    possibleMove[1].push_back(AngleMovement(0, 1, -1,  0, sqrtf(2)));
}


std::vector<AngleMovement> MovementGenerator::getPossibleMove(const unsigned &baseAngle) const
{
    unsigned angle_index = baseAngle % NUMBER_OF_ANGLE_STEPS;
    std::vector<AngleMovement> ret;
    ret.reserve(possibleMove[angle_index].size());

    for(AngleMovement m : possibleMove[angle_index]){
        ret.push_back(m.get_rotate(baseAngle));
    }

    return ret;
}

std::vector<Move3D> MovementGenerator::getPossibleMove3D(const unsigned &baseAngle) const
{
    std::vector<AngleMovement> angleMove = getPossibleMove(baseAngle);

    std::vector<Move3D> ret;
    ret.reserve(angleMove.size());
    for (AngleMovement m : angleMove){
        ret.push_back(m.get_movement());
    }

    return ret;
}

