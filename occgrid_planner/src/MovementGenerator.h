//
// Created by nsix on 3/23/18.
//

#ifndef PROJECT_MOVEMENTGENERATOR_H
#define PROJECT_MOVEMENTGENERATOR_H

#include <vector>
#include <iostream>

#include "AngleMovement.h"

class MovementGenerator {

public:
    MovementGenerator();

    std::vector<AngleMovement> getPossibleMove(const unsigned &baseAngle) const;
    std::vector<Move3D> getPossibleMove3D(const unsigned &baseAngle) const;

private:
    std::vector<AngleMovement> possibleMove[NUMBER_OF_ANGLES_LEVELS];
};


#endif //PROJECT_MOVEMENTGENERATOR_H
