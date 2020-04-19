#include "movementController.hpp"

extern volatile uint8_t debug;

static volatile uint8_t heading_confirmation_required = 0x00;
static volatile uint8_t power_change_required = 0x00;
static volatile uint8_t move_required = 0x00;
static volatile uint8_t turn_required = 0x00;
static volatile uint8_t translation_required = 0x00;

static volatile double desired_move_heading;
static volatile double desired_move_angle;
static volatile double desired_move_velocity;
static volatile double desired_move_distance;

static volatile double desired_turn_heading;
static volatile double desired_turn_angle;
static volatile double desired_turn_velocity;

static volatile int8_t bodyTranslationX = 0;
static volatile int8_t bodyTranslationY = 0;
static volatile int8_t bodyTranslationZ = 0;

//In the current implementation of gait_tripod, we move 5mm with each iteration of the
// step procedure at maximum velocity (and the distance scales linearly with velocity).
//Note: Through experimentation we find that each step is slightly smaller than 5mm... not
// sure if this is due to slippage, bad measurements, not enough timing, or something else.
// Regardless, by making this number smaller, we end up with the right measurement in real
// world applications.  Yay for fudge!
#define STEP_DISTANCE 4.0

#define VEER_CORRECTION_MULTIPLIER 5
#define VEER_CORRECTION_EXPONENT 3

#define HEADING_MARGIN_OF_ERROR 0.05

extern Leg legs[LEG_COUNT];

void walk(Leg legs[])
{

    static uint8_t step_index = 0;

    double veer_correction = 0;

    for (uint8_t legNum = 0; legNum < LEG_COUNT; legNum++)
    {
        Point step = gait_step(legs[legNum], step_index, desired_move_velocity, desired_move_angle, veer_correction);
        step.add(Point(bodyTranslationX, bodyTranslationY, fmin(3, bodyTranslationZ)));
        legs[legNum].setOffset(step);
    }
    step_index++;
    if (step_index > gait_step_count())
    {
        step_index = 0;
    }
}

void translate(Leg legs[])
{
    double bodyTranslationX = 0;
    double bodyTranslationY = 0;
    double bodyTranslationZ = 0;

    for (uint8_t legNum = 0; legNum < LEG_COUNT; legNum++)
    {
        legs[legNum].setOffset(Point(bodyTranslationX, bodyTranslationY, bodyTranslationZ));
    }
}