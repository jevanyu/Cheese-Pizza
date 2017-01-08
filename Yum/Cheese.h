// TODO review EVERYTHING

// add ifdef and ifndef statements


#define CAR_WIDTH 16 //cm

/** DEBUGGING FLAGS */
/*
   If testing on the bench, use 1 so EV_pos is determined by the rear encoder.
   If driving on the ground, use 0 so EV_pos is determined by the front encoder.
*/
#define ENC_DEBUG 0

// Min and max valid target distances
#define MIN_VALID_DIST 890 // cm. give 10cm room in case need to adjust distance at comp.
#define MAX_VALID_DIST 1210 // cm

/* ESC PWM timings: duration of on-pulse in microseconds */
#define ESC_BRAKE 1500
#define ESC_MIN 1000

// 1700 is good
#define ESC_MAX 1700

// esc command to use when coasting
#define ESC_DECEL 1500

// SPRINT acceleration parameters
/*
// vf^2 = 2ax, assuming constant acceleration & v0 =0
// a = vf^2 / 2x
// x = vf^2 / 2a
// vf = max esc pwm = 1700
// use the given acceleration to calculate the accel distance and use that in the diffeq
// for max esc = 1700, then vf = 200 (or is it 160? b/c start at 1540?)
// a = 200pwm*200pwm / 2*300cm = 67 pwm^2/cm = cm/s^2 if pwm = cm/s
// 67 is the parameter used with most success
// 90 doesn't appear to skid, just a few green skid values - see sheet 52
// 100 has a few green skid ratio values - sheet 53
// 120 has visible skid around 5 degrees - sheet 54
// 67 may have slight skid ~ 2 degrees
// banebot wheels
// 50 no skid during accel
// 120 possible skid but no angular skid
*/
#define EV_ACCELERATION 70 // pwm^2/cm

int ACCEL_DIST = (ESC_MAX-ESC_BRAKE)*(ESC_MAX-ESC_BRAKE) / (2*EV_ACCELERATION); // cm to accel in
#define DECEL_DIST 340 // cm before target distance to decel
// on banebot wheels 50 kind of works
// TODO figure out decel dist for banebot wheels and right drag force setting
// #define DECEL_DIST 100

// Sanity checks
#define MAX_OVERSHOOT_ALLOWED 400 // cm
#define MAX_SLIP_ALLOWED 400 // cm
#define MAX_SPRINT_TIME 10 // seconds


// velocity threshold in which to transition from sprint-coast to walk
#define SPRINT_VEL_THRESHOLD 5 // cm/s

// offset from 1500 to use during forward / backward walk
// 28 is good for rubber wheel
// testing 25 on banebot wheels
#define WALK_COMMAND 25 // pwm

// LOG
#define LOG_SIZE 2000  // number of addresses in the memlogging array
#define LOG_PARAMS 8 // num parameters in the array

/** MACROS vehicle state; comments refer to status LED colors */
#define INITIALIZE 0 // all colors
#define WAIT_FOR_DISTANCE 1 // yellow and green
#define WAIT_FOR_LAUNCH 2  //  yellow
#define SPRINT 3 // green
#define WALK 4  // green and red
#define BRAKING 5  // red
#define TURN 6


/** WIRING CONFIGURATION */
/*  Input pins */
// Rear encoder pins
#define PHASE_A_PIN 22
#define PHASE_B_PIN 23
// Front encoder pins
#define FRONT_ENC_PIN1 20
#define FRONT_ENC_PIN2 21
// Microswitch pin
#define SWITCH_PIN 18

/*  Output pins */
#define ESC_PIN 12
// LED pins
#define RED_PIN 8
#define GREEN_PIN 9
#define YELLOW_PIN 10

/** HARDWARE CONSTANTS */

// Back Encoder ticks per rev = COGS_PER_CYCLE * NUM_PHASES
#define COGS_PER_CYCLE 2
#define NUM_PHASES 2
// front encoder ticks per rev
#define ENC_TICKS_PER_REV 192 // front

/* Gearing and wheel/idler radii */
// TODO: measure these values more accurately
#define GEARING (12.0/81.0) // gear ratio: pinion/spur
// #define WHEEL_RAD 2.9675 // 3.005  // rear wheel radius in cm
#define WHEEL_RAD 3.000 // for banebot wheel
// #define IDLER_RAD 1.9718 //1.965 was old idler // wheel radius in cm
#define IDLER_RAD 2.0629

// distances from startline where the photogates are
#define START_TIME 50 // cm
#define END_TIME 850 // cm


/** Conditions for exiting WALK and entering BRAKING state */
// TODO: figure out what the best thresholds are
#define DIST_THRESHOLD 1.0 // acceptable distance error (in cm) for stopping the vehicle
#define VEL_THRESHOLD 10.0 // maximum velocity (in cm/s) for entering BRAKING state

#define VEL_CALC_PERIOD 15 // period (in ms) for calculating velocity

#define IDLER_T_MEMORY_SIZE 20 // number of positions to keep in memory

#define PRINT_PERIOD 10 // delay (in ms) while calling print_state
