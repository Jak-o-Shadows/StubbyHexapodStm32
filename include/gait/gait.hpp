#ifndef GAIT_H
#define GAIT_H

#include <stdlib.h>
#include <stdint.h>

#include "types/Point.hpp"
#include "Leg.hpp"

/*
 * Returns the offset for the given leg at the given step index, scaled by the given velocity
 */
Point gait_step(Leg leg, uint8_t step_index, double linear_velocity, double linear_angle, double rotational_velocity);

/*
 * Reteurns the number of steps in this gait
 */
uint8_t gait_step_count();

#endif
