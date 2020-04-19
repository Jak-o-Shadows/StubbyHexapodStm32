#include "Leg.hpp"

/*
 * Returns the angle which the servo needs to move from neutral.  Used for both tibia and femur.
 */
double solveServoTrapezoid(double desired_angle, double length_a, double length_b, double length_c, double length_d, double angle_E_offset, double angle_E_offset2, double angle_N)
{
	//See diagrams in doc/diagrams.pdf for description of sides and angles
	//Use law of cosines to find the length of the line between the control rod connection point and the servo shaft
	double length_e = sqrt(length_d * length_d + length_a * length_a - 2 * length_d * length_a * cosf(desired_angle + angle_E_offset));

	//Use law of cosines to find the angle between the line we just calculated (e) and the line between the servo shaft and servo horn / control rod connection (b)
	double angle_C = acosf((length_e * length_e + length_b * length_b - length_c * length_c) / (2 * length_e * length_b));
	//Use law of cosines to find the angle between the line we just calculated (e) and the line between the joint and the servo shaft (d)
	double angle_D = acosf((length_e * length_e + length_a * length_a - length_d * length_d) / (2 * length_e * length_a));

#ifdef DEBUG_SIMULATION
	printf("e: %3.1f mm; C: %3.1f°; D: %3.1f°", length_e, angle_C * 180 / M_PI, angle_D * 180 / M_PI);
#endif

	return (angle_C + angle_D + angle_E_offset2) - angle_N;
}

Leg::Leg(uint8_t index, uint8_t tibia_idx, uint8_t femur_idx, uint8_t coxa_idx, double mounting_angle, Point neutralP, uint16_t *servoPos)
{
	this->index = index;
	this->servoIdx[TIBIA] = tibia_idx;
	this->servoIdx[FEMUR] = femur_idx;
	this->servoIdx[COXA] = coxa_idx;
	this->mounting_angle = mounting_angle;
	this->neutralP = neutralP;
	this->servoPos = servoPos;
}

void Leg::setPosition(Point p)
{
	this->p = p;

	//Rotate leg around 0, 0 such that the leg is pointing straight out at angle 0 (straight right).
	p.rotateXY(this->mounting_angle * -1);
	//Translate the leg according to the leg offset, to put the coxa joint at co-ordinates 0,0.
	p.x -= LEG_OFFSET;

	//Find the angle of the leg, used to set the coxa joint.  See figure 2.1, 'coxa angle'.
	double coxa_angle = atan2(p.y, p.x);

	//Find the length of the leg, from coxa joint to end of tibia, on the x,y plane (to be later
	// used for X,Z inverse kinematics).  See figure 2.1, 'leg length', 'p.x', and 'p.y'.
	double leg_length = sqrt((p.x * p.x) + (p.y * p.y));

	//Find the distance between the femur joint and the end of the tibia.  Do this using the
	// right triangle of (FEMUR_HEIGHT + COXA_HEIGHT - z), (leg_length - COXA_LENGTH).  See figure
	// 2.2, 'leg extension'
	double leg_extension = sqrt((FEMUR_HEIGHT + COXA_HEIGHT - p.z) * (FEMUR_HEIGHT + COXA_HEIGHT - p.z) + (leg_length - COXA_LENGTH) * (leg_length - COXA_LENGTH));
	//Find the first part of the femur angle using law of cosines.  See figure 2.2 for a diagram of this.
	double femur_angle_a = acosf((((FEMUR_HEIGHT + COXA_HEIGHT - p.z) * (FEMUR_HEIGHT + COXA_HEIGHT - p.z)) + (leg_extension * leg_extension) - ((leg_length - COXA_LENGTH) * (leg_length - COXA_LENGTH))) / (2 * (FEMUR_HEIGHT + COXA_HEIGHT - p.z) * leg_extension));
	//Find the second part of the femur angle using law of cosines.  See figure 2.3 for a diagram of this.
	double femur_angle_b = acosf(((FEMUR_LENGTH * FEMUR_LENGTH) + (leg_extension * leg_extension) - (TIBIA_LENGTH * TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * leg_extension));
	double femur_angle = femur_angle_a + femur_angle_b;

	//Find the desired tibia angle using law of cosines.  See figure 2.4 for a diagram of this.
	double tibia_angle = acosf(((FEMUR_LENGTH * FEMUR_LENGTH) + (TIBIA_LENGTH * TIBIA_LENGTH) - (leg_extension * leg_extension)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));

#ifdef DEBUG_SIMULATION
	printf(" IK: x,y,z: %d,%d,%d; leg_length: %3.1f mm; leg_extension: %3.1f mm\n", p.x, p.y, p.z, leg_length, leg_extension);
#endif

	this->setTibiaAngle(tibia_angle);
	this->setFemurAngle(femur_angle);
	this->setCoxaAngle(coxa_angle);

	//TODO If the servo angles are out of bounds (either NaN or an integer outside of the valid PWM range), then re-calculate
	// the x,y,z co-ordinates based on valid numbers.  This will be quite doable for out of bounds angles, but will be very
	// difficult for NaN, since it is impossible to say whether it is a small or large angle.
}

uint8_t Leg::getIndex()
{
	return this->index;
}

double Leg::getMountingAngle()
{
	return this->mounting_angle;
}

void Leg::setOffset(Point offset)
{
	Point foot(0, 0, 0);
	foot.x = this->getCalibration(CALIBRATION_X);
	foot.y = this->getCalibration(CALIBRATION_Y);
	foot.z = this->getCalibration(CALIBRATION_Z);
	offset.add(foot);

	if (offset.x > 30)
		offset.x = 30;
	else if (offset.x < -30)
		offset.x = -30;

	if (offset.y > 30)
		offset.y = 30;
	else if (offset.y < -30)
		offset.y = -30;

	if (offset.z > 15)
		offset.z = 15;
	else if (offset.z < -15)
		offset.z = -15;

	offset.add(this->neutralP);
	this->setPosition(offset);
}

void Leg::resetPosition()
{
	this->setOffset(Point(0, 0, 0));
}

Point Leg::getPosition()
{
	return this->p;
}

uint8_t Leg::getServoIdx(uint8_t joint)
{
	return this->servoIdx[joint];
}

int8_t Leg::getCalibration(uint8_t i)
{
	if (i < CALIBRATION_COUNT)
		return this->calibration[i];
	else
		return 0;
}

void Leg::setCalibration(uint8_t i, int8_t calibration)
{
	if (i < CALIBRATION_COUNT)
		this->calibration[i] = calibration;
}

void Leg::setTibiaAngle(double desired_angle)
{
	//See diagrams in doc/diagrams.pdf for description of sides and angles
	double angle_S = solveServoTrapezoid(desired_angle, TIBIA_A, TIBIA_B, TIBIA_C, TIBIA_D, TIBIA_E_OFFSET_ANGLE, TIBIA_E_OFFSET_ANGLE, TIBIA_NEUTRAL_SERVO_ANGLE);
#ifdef DEBUG_SIMULATION
	printf(" Tibia:  desired angle: %3.1f°; servo angle: %3.1f°\n", desired_angle * 180 / M_PI, angle_S * 180 / M_PI);
#endif
	this->servoPos[this->servoIdx[TIBIA]] = (uint16_t)PHASE_NEUTRAL + (this->calibration[TIBIA] * 10) + angle_S * ((PHASE_MAX - PHASE_MIN) / SERVO_TRAVEL);
	//pwm_set_phase_batch((this->index * JOINT_COUNT) + TIBIA, (uint16_t)PHASE_NEUTRAL + (this->calibration[TIBIA] * 10) + angle_S * ((PHASE_MAX - PHASE_MIN) / SERVO_TRAVEL));
}

void Leg::setFemurAngle(double desired_angle)
{
	//See diagrams in doc/diagrams.pdf for description of sides and angles
	double angle_S = solveServoTrapezoid(desired_angle, FEMUR_A, FEMUR_B, FEMUR_C, FEMUR_D, FEMUR_E_OFFSET_ANGLE, FEMUR_E_OFFSET_ANGLE, FEMUR_NEUTRAL_SERVO_ANGLE);
#ifdef DEBUG_SIMULATION
	printf(" Femur:  desired angle: %3.1f°; servo angle: %3.1f°\n", desired_angle * 180 / M_PI, angle_S * 180 / M_PI);
#endif
	this->servoPos[this->servoIdx[FEMUR]] = (uint16_t)PHASE_NEUTRAL + (this->calibration[FEMUR] * 10) + angle_S * ((PHASE_MAX - PHASE_MIN) / SERVO_TRAVEL);
	//pwm_set_phase_batch((this->index * JOINT_COUNT) + FEMUR, (uint16_t)PHASE_NEUTRAL + (this->calibration[FEMUR] * 10) + angle_S * ((PHASE_MAX - PHASE_MIN) / SERVO_TRAVEL));
}

void Leg::setCoxaAngle(double desired_angle)
{
	this->servoPos[this->servoIdx[COXA]] = (uint16_t)PHASE_NEUTRAL + (this->calibration[COXA] * 10) + desired_angle * COXA_PHASE_MULTIPLIER;
	//pwm_set_phase_batch((this->index * JOINT_COUNT) + COXA, (uint16_t)PHASE_NEUTRAL + (this->calibration[COXA] * 10) + desired_angle * COXA_PHASE_MULTIPLIER);
}
