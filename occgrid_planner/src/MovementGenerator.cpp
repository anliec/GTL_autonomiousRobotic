//
// Created by nsix on 3/23/18.
//

#include "MovementGenerator.h"

MovementGenerator::MovementGenerator()
{
    //set possible movement for angle = 0 / 90 / 180 / 270
    possibleMove[0].emplace_back(0, 0,  0,  1, 1.0f); // rotate on the spot
    possibleMove[0].emplace_back(0, 1, -1,  2, 0.0f); // diagonal + 90 degree
    possibleMove[0].emplace_back(0, 2, -1,  1, 0.0f); // diagonal + 1x + 45 degree
    //symmetrical
    possibleMove[0].emplace_back(0, 0,  0, -1, 1.0f); // rotate on the spot
    possibleMove[0].emplace_back(0, 1,  1, -2, 0.0f); // diagonal + 90 degree
    possibleMove[0].emplace_back(0, 2,  1, -1, 0.0f); // diagonal + 1x + 45 degree
    //add forward movement
    possibleMove[0].emplace_back(0, 1,  0,  0, 0.0f);

    //set possible movement for angle = 45 / 135 / 225 / 315
    possibleMove[1].emplace_back(0, 0,  0,  1, 1.0f); // rotate on the spot
    possibleMove[1].emplace_back(0, 1,  0, -2, 0.0f); // diagonal + 90 degree
    possibleMove[1].emplace_back(0, 2, -1, -1, 0.0f); // diagonal + 1x + 45 degree
    //symmetrical
    possibleMove[1].emplace_back(0, 0,  0, -1, 1.0f); // rotate on the spot
    possibleMove[1].emplace_back(0, 0, -1,  2, 0.0f); // diagonal + 90 degree
    possibleMove[1].emplace_back(0, 1, -2,  1, 0.0f); // diagonal + 1y + 45 degree
    //add forward movement
    possibleMove[1].emplace_back(0, 1, -1,  0, 0.0f);

    //generate movement for angles  90 / 180 / 270
    for(unsigned a=2 ; a<NUMBER_OF_ANGLES_LEVELS ; a+=2){
        for(AngleMovement m : possibleMove[0]){
            possibleMove[a].push_back(m.get_rotate(a));
        }
    }
    //generate movement for angles  135 / 225 / 315
    for(unsigned a=3 ; a<NUMBER_OF_ANGLES_LEVELS ; a+=2){
        for(AngleMovement m : possibleMove[1]){
            possibleMove[a].push_back(m.get_rotate(a));
        }
    }

}

std::vector<AngleMovement> MovementGenerator::getPossibleMove(const unsigned &baseAngle) const
{
    assert(baseAngle < NUMBER_OF_ANGLES_LEVELS);
    return possibleMove[baseAngle];
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

