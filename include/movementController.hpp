#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <stdint.h>

#include "Leg.hpp"
#include "gait/gait.hpp"

void walk(Leg legs[]);
void translate(Leg legs[]);

#endif