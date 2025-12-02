// HEADER GUARDS

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

// QUANTITIES

#define PI 3.14159

#define TILE 23.622

#define TRACKING_WHEEL_DIAMETER 2.04693272372 * 0.989350176396 * 0.985034552924 
#define WHEEL_DIAMETER 3.25

#define WHEELBASE 11.375
#define DRIVETRAIN_RPM 480
#define HORIZONTAL_DRIFT 4

#define PARK_ZONE_WIDTH 18.87
#define PARK_ZONE_DEPTH 16.86

#define TRACKING_CENTER_DISTANCE_FROM_BACK 7.75
    // 31 holes from front to back, 27 holes from left to right
    // 31/2 = 15.5 inches from front to back, 27/2 = 13.5 inches from left to right (1 hole = 0.5 inches)
    // 15.5/2 = 7.75 inches from front/back to tracking center, 13.5/2 = 6.75 inches from left/right to tracking center
#define TRACKING_CENTER_DISTANCE_FROM_LEFT 6.75

#define HORIZONTAL_ENC_DISTANCE -0.75
    // 14 holes from back
    // 7 inches from back
    // distance from back - distance of tracking center from back = 7 - 7.75 = -0.75 inches FORWARD from tracking center 
#define VERTICAL_ENC_DISTANCE -1.75
    // 12 holes from left
    // 6 holes from left
    // distance from left - tracking center = 6 - 7.75 = -1.75 inches RIGHT of (i.e. 1.75 inches LEFT) tracking center

// OTHER LIBRARIES

#include "main.h"

#include "subsystems\declarations.hpp"
#include "subsystems\mechanisms.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// FORWARD DECLARATIONS

extern command mechanism_state_var;

enum class autonomous_selection : int {test, middle_control};

void execute_autonomous(autonomous_selection slct);

extern int odom_alive;
void arc_odometry_fn();

extern double correction;

void turning_PID (double desired_degrees, 
                  double scaling = 1,
                  double k_P = 2, 
                  double k_I = 0, 
                  double k_D = 6, 
                  double anti_windup = 3);
void straight_PID (double desired_x, 
                   double desired_y, 
                   bool is_going_backwards = false,
                   double scaling = 1,
                   double k_P_distance = 4, 
                   double k_I_distance = 0, 
                   double k_D_distance = 0.5, 
                   double anti_windup_distance = 3);
                   
extern double pose_x;
extern double pose_y;

extern double delta_pose_x;
extern double delta_pose_y;

extern double pose_x_previous;
extern double pose_y_previous;
extern double pose_theta_previous;

// SNAPSHOT ALL INPUTS (so that when values change in the middle it does not matter)
    extern double theta_raw;
    extern double verticalEnc_raw;
    extern double horizontalEnc_raw;

extern double s_S;
extern double s_R;

extern double pose_S_previous;
extern double pose_R_previous;
extern double pose_theta_previous;

extern double delta_S;
extern double delta_R;
extern double delta_theta;

extern double r_S; // Distance to center of rotation for perpendicular (horizontal) component of movement
extern double r_R; // Distance to center of rotation for vertical component of movement
extern double r_A;

double exponential_drive(double input, double controllerDeadband = 3, double drivetrainDeadband = 10, double exponential_gain = 1.019);

#endif  // AUTONOMOUS_H