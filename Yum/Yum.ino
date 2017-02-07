/*
   AUTHOR: Andrew Shieh, Jevan Yu
   PURPOSE: run fast zoom zoom
*/

#include <PID_v1.h>
#include <Servo.h>
#include <Encoder.h>
#include "Cheese.h"

/*                                *
   ONLY CHANGE THIS WHEN RUNNING
 *                                */
int target_distance = 900; //distance from start pt to end pt (cm)

// Data log
unsigned short data_log[LOG_SIZE][LOG_PARAMS];
unsigned short log_index = 0; //counter for data_log

Servo esc;
Encoder drive(PHASE_A_PIN, PHASE_B_PIN);  //hall-effect encoder inside the motor
Encoder idler(FRONT_ENC_PIN1, FRONT_ENC_PIN2); //CUI capacitive encoder on the front axle

/******** Machine states *********/
int current_state = INITIALIZE;
int prev_state = INITIALIZE;

/******** Time variables *********/
unsigned long last_time = 0; //time (in ms) of the start of the last loop
unsigned long current_time = 0; //time (in ms) of the start of the current loop since the program started
unsigned long elapsed_time = 0; //time (in ms) between start of last loop and beginning of current

//time variables related to SPRINT
unsigned long start_time = 0; //time (in ms) at which SPRINT was entered
unsigned long time_since_start = 0; //time (in ms) since start of SPRINT

unsigned long start_run_time = 0; //time when sprint started
unsigned long current_run_time = 0; //time since vehicled passed 0.5m
unsigned long final_run_time = 0; //time when vehicle passed 8.5m since hit 0.5m

unsigned long last_vel_calc = 0; //time (in ms) of last velocity calculation
unsigned long last_print = 0; //time (in ms) of last print

int true_distance; //true distance it travels
int start_slowing_dist;
int start_walk_dist;

/******* Vehicle variables ********/

// theta (angular position)
double idler_t = 0.0; //rad
double last_idler_t = 0.0;

double drive_t = 0.0; //rad
double last_drive_t = 0.0;

// omega (angular velocity)
double idler_w = 0.0; //rad/s
double last_idler_w = 0.0;

double slip_ratio = 1.0; //drive_w / idler_w
// due to double precision errors even when drive_vel = EV_vel, slip_ratio might not be 1
// If back wheels skid on acceleration (turning more than front) then slip ratio > 1
// If front wheels skid on deceleration (turning less than front) then slip ratio < 1
// tl;dr: > 1 if slips during accel, < 1 if slips during decel

double drive_w = 0.0; //rad/s
double last_drive_w = 0.0;

// distance and position
double drive_d = 0; //cm

double vehicle_pos = 0.0; //cm
double last_pos = 0.0; //cm

double previous_idler_thetas[IDLER_T_MEMORY_SIZE];
double previous_drive_thetas[IDLER_T_MEMORY_SIZE];
double previous_idler_thetas_timestamps[IDLER_T_MEMORY_SIZE];

// Velocity
double drive_vel = 0; //cm/s
float vehicle_vel = 0.0; //put a low pass on this guy

// ESC command
int esc_command = 1000;

/******** Methods begin here *********/

// assumes vehicle takes a straight path to the cans, then curves to the final point
void calculate_true_distance() {
  double a = 100 - CAN_WIDTH; //aims for middle of cans
  double b = target_distance /2; //half of the assigned distance
  
  true_distance = PI *sqrt((pow(a,2) + pow(b,2))/2); //curve from cans to final
                                                      // calculated as 1/2 * circumference of ellipse
}


void calculate_vehicle_velocity_avg() {
  //in ms
  double time_interval = (previous_idler_thetas_timestamps[0] - previous_idler_thetas_timestamps[IDLER_T_MEMORY_SIZE - 1]);

  //converts rad/s to rad/ms
  idler_w = (previous_idler_thetas[0] - previous_idler_thetas[IDLER_T_MEMORY_SIZE - 1]) * 1000 / time_interval;
  drive_w = (previous_drive_thetas[0] - previous_drive_thetas[IDLER_T_MEMORY_SIZE - 1]) * 1000 / time_interval;

  drive_vel = drive_w * WHEEL_RAD;

  if (ENC_DEBUG)
    vehicle_vel = drive_vel;
  else
    vehicle_vel = idler_w * IDLER_RAD;

  slip_ratio = drive_vel / vehicle_vel;
}

/* Assumptions: the stack is an array with index 0 as the top of the stack */
void push_value_to_stack(double target_array[], int array_size, double value) {

  // pushes all values down one index
  for (int i = array_size - 1; i >= 1; i--) {
    target_array[i] = target_array[i - 1];
  }

  //adds new value to the top of stack
  target_array[0] = value;
}

void calculate_vehicle_position() {
  /*
   * Reads the encoder values and calculates the drive and idler positions
   */
  drive_t = 2 * PI * (double) drive.read() / (COGS_PER_CYCLE * NUM_PHASES);
  drive_t = GEARING * drive_t;

  idler_t = 2 * PI * (double) idler.read() / (ENC_TICKS_PER_REV);

  // push new idler theta and drive theta into the memory stack
  push_value_to_stack(previous_idler_thetas,IDLER_T_MEMORY_SIZE,idler_t);
  push_value_to_stack(previous_drive_thetas, IDLER_T_MEMORY_SIZE, drive_t);

  // TODO may run into problem casting unsigned long current_time to double
  push_value_to_stack(previous_idler_thetas_timestamps,IDLER_T_MEMORY_SIZE,(double)current_time);

  //Serial.println("prev drive t: " + String(previous_drive_thetas[0]));
  
  vehicle_pos = drive_t * WHEEL_RAD;
  drive_d = drive_t * WHEEL_RAD;
  //vehicle_pos = idler_t * IDLER_RAD;

  // DEBUG DEBUG DEBUG DEBUG
  if(ENC_DEBUG) {
    vehicle_pos = drive_d;
  }

  //double K = 1.0; // 1.0 means no weight to past, 0 is dumb, 0.7 prob good
  //vehicleVelocity = K*measuredVelocity + (1-K)*vehicleVelocity; // low pass filter to smooth velocity
}

/***** ALL METHODS BELOW THIS LINE UNTIL SETUP() ARE COPY-PASTED *****/

// supposedly changes LEDs to match state
void display_state_LED(int s) {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  switch (s) {
    case INITIALIZE:
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(YELLOW_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);
      break;
    case SPRINT:
      digitalWrite(GREEN_PIN, HIGH);
      break;
    case WALK:
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(YELLOW_PIN, HIGH);
      break;
    case BRAKING:
      digitalWrite(RED_PIN, HIGH);
      break;
    case WAIT_FOR_DISTANCE:
      digitalWrite(YELLOW_PIN, HIGH);
      break;
    case WAIT_FOR_LAUNCH:
      digitalWrite(YELLOW_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);
      break;
    default:
      break;
  }
}

// supposedly returns the vehicle's current state ¯\_(ツ)_/¯
String state_to_string(int s) {
  switch (s) {
    case INITIALIZE:
      return "initialize";
      break;
    case SPRINT:
      return "sprint";
      break;
    case WALK:
      return "walk";
      break;
    case BRAKING:
      return "braking";
      break;
    case WAIT_FOR_DISTANCE:
      return "wait for distance";
      break;
    case WAIT_FOR_LAUNCH:
      return "wait for launch";
      break;
    default:
      return "";
  }
}

// supposedly logs state to memory?
void log_state() {
  /*
     SUPER JANKY CODE FOR LOGGING TO MEMORY
  */
  // array to log certain values throughout the run

  /*
     Each time this function is called, it logs the current values of:
     current_state, esc_command, vehicle_pos, drive_d, time_since_start
     as unsiged shorts to the next available addresses in the log array.
  */
  if (log_index >= LOG_SIZE - 10) {  // full with 10 free spaces for safety
    Serial.println("Log full!");
  }
  else {
    if (current_state == WAIT_FOR_LAUNCH) return;
    if (current_state == WAIT_FOR_DISTANCE) return;
    if (current_state == BRAKING) return;
    data_log[log_index][0] = (unsigned short)current_state;
    data_log[log_index][1] = (unsigned short)esc_command;
    data_log[log_index][2] = (unsigned short)vehicle_pos;
    data_log[log_index][3] = (unsigned short)drive_d;
    data_log[log_index][4] = (unsigned short)vehicle_vel;
    data_log[log_index][5] = (unsigned short)current_run_time;
    data_log[log_index][6] = (unsigned short)time_since_start;
    data_log[log_index][7] = (unsigned short)(slip_ratio * 1000); // dont get confused by multiplier!
  }
  log_index++;
}

int max_in_log(unsigned short log[][LOG_PARAMS], int index) {
  int max = 0;
  for (int i = 0; i < (int)log_index; i++) {
    max = max(log[i][index], max);
  }
  return max;
}

int array_max(unsigned short log[], int array_size) {
  int max = 0;
  for (int i = 0; i < array_size; i++) {
    max = max((int)log[i], max);
  }
  return max;
}

void calc_slip(unsigned short log[][LOG_PARAMS], unsigned short slip[]) {
  for (int i = 0; i < (int)log_index; i++) {
    slip[i] = data_log[i][2] - data_log[i][3];
  }
}

// NOT SURE IF RESETS EVERYTHING
void reset_vehicle_state() {
  drive.write(0);
  idler.write(0);
  vehicle_pos = 0;
  vehicle_vel = 0;
  drive_vel = 0;
  drive_d = 0;

  idler_t = 0.0; // rads
  last_idler_t = 0.0;

  drive_t = 0.0; //rads
  last_drive_t = 0.0;

  last_pos = 0;

  for (int i = 0; i < IDLER_T_MEMORY_SIZE; i++) {
    previous_idler_thetas[i] = 0;
  }
}



/******* Start of SETUP and LOOP *******/
void setup() {
  // put your setup code here, to run once: ok

  Serial.begin(115200);
  
  Serial.println("Starting...");
  last_time = millis();
  esc.attach(ESC_PIN);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  display_state_LED(current_state);

  reset_vehicle_state();

  delay(2000);
}


void loop() {  
  // time calculations
  current_time = millis();
  elapsed_time = current_time - last_time;
  last_time = current_time;

  // calculate vehicle state
  calculate_vehicle_position();
  calculate_vehicle_velocity_avg();

  /* SAFETY CHECK: goes into brake mode if elapsed time > 20s  */
  // TODO fine tune time needed
  if (time_since_start > RUN_TIME * 1000 && !ENC_DEBUG) {
    current_state = BRAKING;
    Serial.println("Total time since start is more than " + String(RUN_TIME) + " seconds");
  }


  switch (current_state) {
    case INITIALIZE:
      {
        Serial.println("Initialization done");
        Serial.print("target distance: " + String(target_distance));   

        calculate_true_distance();

        Serial.println(", true distance: " + String(true_distance));
             
        esc_command = ESC_BRAKE; //brake

        start_slowing_dist = min(true_distance - DECEL_DIST, END_TIME);
        start_walk_dist = start_slowing_dist + DECEL_DIST;

        Serial.println(start_slowing_dist); Serial.println(start_walk_dist);

        
        current_state = WAIT_FOR_LAUNCH;
        break;
      }
    case WAIT_FOR_LAUNCH:
      {
        esc_command = ESC_BRAKE; //brake

        // polls the microswitch to see if should launch
        int switch_state = digitalRead(SWITCH_PIN);
        if (switch_state == LOW) {                    // high might pose a problem
          Serial.println("Sprinting in 3 seconds...");

          // delay creates an annoying problem where current_time isn't updated
          delay(3000);

          current_time = millis();
          start_time = current_time;

          reset_vehicle_state();

          current_state = SPRINT;

        }
        break;
      }

    case SPRINT:
      {
        // time since start of sprinting
        time_since_start = current_time - start_time;

        // if just passed start time distance set start time
        if (vehicle_pos > START_TIME && start_run_time == 0) {
          start_run_time = current_time;
        }
        // if just passed end time distance set stop time
        if (vehicle_pos > END_TIME && final_run_time == 0)  {
          final_run_time = current_time;
        }
        // if in between, update current run time
        if (vehicle_pos > START_TIME && vehicle_pos < END_TIME) {
          current_run_time = current_time - start_run_time;
        }


        /***** ACCELERATION SUB STATE *****/
        if (vehicle_pos < ACCEL_DIST) { //still accelerating to max
          esc_command = map(sqrt(vehicle_pos), 0, sqrt(ACCEL_DIST), ESC_BRAKE + 40, ESC_MAX);
                      
          // by making v(t) =  k*sqrt(x(t)) then x(t) = a*t^2
          // maps 0 to accel dist to 1540 to max esc, MIGHT SKID OUT

        }
        /***** CONSTANT MAX SPEED SUB STATE *****/
        else if (vehicle_pos >= ACCEL_DIST && vehicle_pos < start_slowing_dist) {
          esc_command = ESC_MAX;
        }

        /***** DECEL SUB STATE *****/
        else if (vehicle_pos >= start_slowing_dist) {
          /**** TRANSITION TO WALK *****/
          // If in the decel period but you're stopped then go to walk mode
          // DANGER: The distance the car will overshoot/travel during decel is totally
          // unpredictable!! Car could travel too far if target is 9m
          // DANGER: If vehicle_vel is incorrectly calculated / there is an error, vehicle
          // may transition to walk prematurely
          if (vehicle_vel < SPRINT_VEL_THRESHOLD) {
            current_state = WALK;
          }
          else {
            if (vehicle_pos < start_slowing_dist + 50) {
              esc_command = map(vehicle_pos, start_slowing_dist, start_slowing_dist + 50, ESC_MAX, ESC_DECEL);
            }
            else {
              // coast with drag brake command
              esc_command = ESC_DECEL;
            }
          }
        }
        /******* TRANSITION TO WALK SUB STATE ******/
        else { //shouldn't get here with coast to stop logic
          current_state = WALK;
          esc_command = ESC_BRAKE;
        }

        /***** SAFETY CHECKS *****/
        // Idler check
        if (vehicle_pos > true_distance + MAX_OVERSHOOT_ALLOWED) {
          current_state = BRAKING;
          Serial.println("vehicle_pos > " + String(true_distance + MAX_OVERSHOOT_ALLOWED));
        }

        // Check wheel (drive) position
        if (drive_d > true_distance + MAX_OVERSHOOT_ALLOWED) {
          current_state = BRAKING;
          Serial.println("drive_d > " + String(true_distance + MAX_OVERSHOOT_ALLOWED) + ". slipping");
        }

        // Check if time is too much
        if (time_since_start > MAX_SPRINT_TIME * 1000) {
          current_state = BRAKING;
          Serial.println("sprinting more than " + String(MAX_SPRINT_TIME) + "s.");
        }

        if (abs(vehicle_pos - drive_d) > MAX_SLIP_ALLOWED) {
          current_state = BRAKING;
          Serial.println("Diff in vehicle_pos and drive_d > 100.");
        }

        break;
      }

    // TODO change target_distance to true_distance
    case WALK:
      {
        // time since start of sprinting
        time_since_start = current_time - start_time;

        // if going fast, like right out of sprint, then brake hella (takes ~ 90ms to switch into reverse)
        if (vehicle_vel > 500) {
          Serial.println("break break break break");
          esc_command = ESC_MIN;
        }

        // if close to target and velocity isn't much then brake
        else if (abs(vehicle_pos - true_distance) < DIST_THRESHOLD
                 && abs(vehicle_vel) < VEL_THRESHOLD) {
          current_state = BRAKING;
        }

        // this year, you can't go backwards
        else {
          if (vehicle_pos < true_distance - DIST_THRESHOLD) {

            Serial.println("pos " + String(vehicle_pos)); Serial.println(String(true_distance - DIST_THRESHOLD));
            esc_command = ESC_BRAKE + WALK_COMMAND;

            Serial.println("esc " + String(esc_command));
          }
          else {
            // went over
            esc_command = ESC_BRAKE;
          }
        }

        /***** WALK SAFETY CHECKS *****/

        if (abs(vehicle_pos - drive_d) > MAX_SLIP_ALLOWED) {
          current_state = BRAKING;
          Serial.println("Diff in EV_pos and drive_d > " + String(MAX_SLIP_ALLOWED) + ".");
        }

        if (vehicle_pos > true_distance + MAX_OVERSHOOT_ALLOWED) {
          current_state = BRAKING;
          Serial.println("EV_pos > " + String(true_distance + MAX_OVERSHOOT_ALLOWED) + " during walk.");
        }

        // Check wheel (drive) position
        if (drive_d > true_distance + MAX_OVERSHOOT_ALLOWED) {
          current_state = BRAKING;
          Serial.println("drive_d > " + String(true_distance + MAX_OVERSHOOT_ALLOWED) + " during walk./ slipping");
        }

        break;
      }

    case BRAKING:
      {
        esc_command = ESC_BRAKE;

        // necessary hack to ensure the esc is sent the brake command before the delay (10 seconds)
        esc.writeMicroseconds(esc_command);

        /* testing detaching esc */
        delay(1000);
        esc.detach();
        delay(10000);  //wait 10 seconds between consecutive prints of the log
        break;

      }

    default:
      break;
  }

  // write the esc command
  esc.writeMicroseconds(esc_command);

  // Print state if it's in a new position or new state and if enough time has passed since last print
  if ((vehicle_pos != last_pos || prev_state != current_state) && (current_time - last_print) > PRINT_PERIOD) {
    Serial.println(state_to_string(current_state));
    Serial.println(String(vehicle_pos));
    
    log_state();
    last_print = current_time;
  }
  if (prev_state != current_state) {
    display_state_LED(current_state);
  }

  // update previous state and position
  prev_state = current_state;
  last_pos = vehicle_pos;


  
  // arbitrary delay so teensy can send serial
  // teensy is so fast that loop time w/o delay is < 0.5 ms
  delay(1);

}

