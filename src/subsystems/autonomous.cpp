#include "main.h"

#include "subsystems\autonomous.hpp"
#include "subsystems\mechanisms.hpp"
#include "subsystems\declarations.hpp"

#include <math.h>

// Define functions

command mechanism_state_var;

double pose_x = 0;
double pose_y = 0;

double delta_pose_x = 0;
double delta_pose_y = 0;

double pose_x_previous = 0;
double pose_y_previous = 0;
double pose_theta_previous;

// SNAPSHOT ALL INPUTS (so that when values change in the middle it does not matter)
    double theta_raw;
    double verticalEnc_raw;
    double horizontalEnc_raw;

double s_S = HORIZONTAL_ENC_DISTANCE;
double s_R = VERTICAL_ENC_DISTANCE;

double pose_S_previous = 0;
double pose_R_previous = 0;

double delta_S;
double delta_R;
double delta_theta;

double r_S; // Distance to center of rotation for perpendicular (horizontal) component of movement from vertical encoder
double r_R; // Distance to center of rotation for vertical component of movement from horizontal encoder
double r_A; // Distance to center of rotation (same as r_R) for vertical component of movement from tracking center

int odom_alive = 0;
void arc_odometry_fn ()
{

    pose_theta_previous = 0; // Change using GPS sensor
    horizontalEnc.reset_position(); // resets position to 0 (reset method only resets it to current angle of rotation sensor)
    verticalEnc.reset_position();

    while (true)
    {
        verticalEnc_raw = (double)verticalEnc.get_position()/100 * PI/180 * TRACKING_WHEEL_DIAMETER/2;  // centidegrees --> degrees --> radians --> inches
        horizontalEnc_raw = (double)horizontalEnc.get_position()/100 * PI/180 * TRACKING_WHEEL_DIAMETER/2;
        theta_raw = imu.get_heading() * PI/180; // degrees --> radians

        delta_S = verticalEnc_raw - pose_S_previous;
        delta_R = horizontalEnc_raw - pose_R_previous;
        delta_theta =  theta_raw - pose_theta_previous; 

        r_S = delta_S/delta_theta + s_S;
        r_R = delta_R/delta_theta + s_R;

        r_A = r_R - s_R;
        
        // Shifted coordinate frame: let the y-axis be the direction of movement
        // Let the x-axis be the horizontal component
            // x-component: 2 * sin (delta_theta/2) * r_S
            // y-component: 2 * sin (delta_theta/2) * r_A
        // The shifted coordinate frame is offset by pose_theta_previous + delta_theta/2 (why?) from the global frame

        // Rotate everything by -(pose_theta_previous + delta_theta/2) (i.e. go back in that direction)
        // Can calculate from rotation matrix

        // Wrong
            // delta_pose_x = 2 * sin (delta_theta/2) * ( r_S * cos (-(pose_theta_previous + delta_theta/2)) - r_A * sin (-(pose_theta_previous + delta_theta/2)) );
            // delta_pose_y = 2 * sin (delta_theta/2) * ( r_S * sin (-(pose_theta_previous + delta_theta/2)) + r_A * cos (-(pose_theta_previous + delta_theta/2)) );

        if (fabs(delta_theta) < 1e-6)
        {
            delta_pose_y = delta_S;
            delta_pose_x = delta_R;
        }
        else
        {
            delta_pose_x = 2 * sin (delta_theta/2) * ( r_S * sin (-(pose_theta_previous + delta_theta/2)) - r_A * cos (-(pose_theta_previous + delta_theta/2)) );
            delta_pose_y = -2 * sin (delta_theta/2) * ( r_S * cos (-(pose_theta_previous + delta_theta/2)) + r_A * sin (-(pose_theta_previous + delta_theta/2)) );
        }
        
        pose_x += delta_pose_x; // Should be 0 if nothing happens
        pose_y += delta_pose_y;

        // Update previous (these values are not used for the rest of the loop so it's fine to do it here)
                pose_S_previous = verticalEnc_raw; // centidegrees --> degrees --> radians
                pose_R_previous = horizontalEnc_raw;

                pose_theta_previous = theta_raw; // degrees --> radians

        // Delay
        odom_alive += 20;
        // pros::lcd::print(5, "odometry alive for: %f msec", odom_alive);
        // I can't be randomly trying to print to the lcd outside of the lcd printing task
        pros::delay(20);
    }
}

double correction = 0;

void turning_PID (double desired_degrees, double scaling, double k_P, double k_I, double k_D, double anti_windup) // turns directly to the heading desired
{
    double theta_raw_PID = imu.get_heading(); // stay in degrees

    // Normalize theta_raw_PID itself
        while (theta_raw_PID > 180) theta_raw_PID -= 360;
        while (theta_raw_PID < -180) theta_raw_PID += 360;

    double error = desired_degrees - theta_raw_PID; 
        // if desired_degrees > theta_raw_PID, this is positive (positive = counterclockwise)
        // if desired_degrees < theta_raw_PID, this is negative (negative = clockwise)
    
    // Normalize error
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

    double past_error = error;
    
    double integral = 0;

    double gyro_rate = 0;
    double derivative = 0;
    
    int time_correct = 0; // ms
    int safety_timeout = 0;
    
    while (time_correct < 100 && safety_timeout < 5000) // settle time of 100 ms
    {

        correction = scaling * (k_P * error + k_I * integral + k_D * derivative);

        left_mg.move(k_P * error + k_I * integral + k_D * derivative); // should move forward if the error is positive
        right_mg.move(-(k_P * error + k_I * integral + k_D * derivative)); // should move backward if the error is negative

        integral += error * 0.02;
        if (fabs(integral) > anti_windup)
        {
            double sign_integral;
            if (integral > 0)
            {
                sign_integral = 1;
            }
            else
            {
                sign_integral = -1;
            }
            integral = sign_integral * anti_windup; // remember, the integral correction is integral * k_I
        }

        gyro_rate = imu.get_gyro_rate().z; // degrees per second
        derivative = -gyro_rate; // negative in order to slow down the bot

        past_error = error;

        theta_raw_PID = imu.get_heading();
        
        // Normalize theta_raw_PID itself
            while (theta_raw_PID > 180) theta_raw_PID -= 360;
            while (theta_raw_PID < -180) theta_raw_PID += 360;

        error = desired_degrees - theta_raw_PID;
        
        // Normalize error
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

        if (fabs(error) < 3)
        {
            time_correct += 20;
        }
        else
        {
            time_correct = 0;
        }
        
        safety_timeout += 20;
        pros::delay(20);
    }

    left_mg.move(0);
    right_mg.move(0);
}

void straight_PID (double desired_x, double desired_y, bool is_going_backwards, double scaling, double k_P_distance, double k_I_distance, double k_D_distance, double anti_windup_distance)
{
    double original_x = pose_x;
    double original_y = pose_y;

    double desired_theta = atan2(desired_y - pose_y, desired_x - pose_x); // radians -- all trig in c is in radians
        // angle from current position to desired position (used for calculating signed distance_error)
    double distance_error = (desired_x - pose_x)*cos(desired_theta) + (desired_y - pose_y)*sin(desired_theta); // if this is confusing think of writing desired_x - pose_x as hypotenuse*cos(desired_theta)

    double line_length = sqrt(pow(desired_x - original_x, 2) + pow(desired_y - original_y, 2));
    
    double past_distance_error = distance_error;

    double distance_derivative_1 = 0;
    double distance_derivative_2 = 0;
    double distance_derivative_3 = 0;

    double smoothed_distance_derivative = 0;

    double distance_integral = 0;

    double distance_correction;

    int time_correct = 0; // ms
    int safety_timeout = 0;

    while (time_correct < 100 && safety_timeout < 1000)
    {
        // Calculate raw errors, signed
            desired_theta = atan2(desired_y - pose_y, desired_x - pose_x);
            distance_error = (desired_x - pose_x)*cos(desired_theta) + (desired_y - pose_y)*sin(desired_theta);

        // Pass and calculate derivatives
            distance_derivative_3 = distance_derivative_2; // 2 --> 3
            distance_derivative_2 = distance_derivative_1; // 1 --> 2
            distance_derivative_1 = (distance_error - past_distance_error)/0.02; // 20 ms = 0.02 s
            smoothed_distance_derivative = (distance_derivative_1 + distance_derivative_2 + distance_derivative_3)/3;

        // Calculate integral
            distance_integral += distance_error * 0.02; // 20 ms = 0.02 s

        // Clamp integral --> anti-windup
            if (fabs(distance_integral) > anti_windup_distance)
            {
                double sign_distance_integral;
                if (distance_integral > 0)
                {
                    sign_distance_integral = 1;
                }
                else
                {
                    sign_distance_integral = -1;
                }
                distance_integral = sign_distance_integral * anti_windup_distance; // remember, the integral correction is integral * k_I
            }

        // Calculate corrections
            distance_correction = scaling * (k_P_distance * distance_error + k_I_distance * distance_integral + k_D_distance * smoothed_distance_derivative);
            if (is_going_backwards) 
            {
                distance_correction = -distance_correction;
            }

        // Update storage of past errors
            past_distance_error = distance_error;

        // Move the bot
            left_mg.move(distance_correction); 
            right_mg.move(distance_correction);

        // Update exit conditions + delay
            if (fabs(distance_error) < 1)
            {
                time_correct += 20;
            }
            else
            {
                time_correct = 0;
            }

            safety_timeout += 20;

            pros::delay(20);
    }

    left_mg.move(0);
    right_mg.move(0);
}

void mechanism_state_fn()
{
    pros::lcd::print(0, "Mech task");
    while (true)
    {
        // collecting, low, middlea, middleb, higha, highb, stop
        switch (mechanism_state_var)
        {
            case command::collecting:
                execute_command(command::collecting);
                pros::lcd::print(4, "command::collecting");
                break;

            case command::low:
                execute_command(command::low);
                pros::lcd::print(4, "command::low");
                break;

            case command::middle:
                execute_command(command::middle);
                pros::lcd::print(4, "command::middle");
                break;

            case command::high:
                execute_command(command::high);
                pros::lcd::print(4, "command::high");
                break;

            case command::stop:
                execute_command(command::stop);
                pros::lcd::print(4, "command::stop");
                break;
            }

            pros::delay(20);
        }
    }

void execute_autonomous(autonomous_selection slct) 
{
    // pros::Task mechanism_state_task(mechanism_state_fn); // create outside of switch statement

    switch (slct)    
    {
        case (autonomous_selection::test):
            // mechanism_state_var = command::low;
            outtake_value = !outtake_value;
			outtake_pneumatics.set_value(outtake_value);
            pros::lcd::print(6, "outtake_value: %d", outtake_value);
            park_value = !park_value;
			park.set_value(park_value);
            descore_value = true;
			descore.set_value(descore_value);
            outtake_value = !outtake_value;
			outtake_pneumatics.set_value(outtake_value);
            pros::lcd::print(7, "outtake_value: %d", outtake_value);
            break;

        case (autonomous_selection::middle_control):
            mechanism_state_var = command::collecting;
            outtake_value = false;
			outtake_pneumatics.set_value(outtake_value);
			pros::delay(170);

            // (0) Get ready
            straight_PID(0, -TILE + TRACKING_CENTER_DISTANCE_FROM_BACK);
            turning_PID(90);

            // (1) Collect balls
            straight_PID(TILE + 1, -TILE + TRACKING_CENTER_DISTANCE_FROM_BACK);
            mechanism_state_var = command::low;
            // pros::delay(500);
            mechanism_state_var = command::collecting;
            // pros::delay(500);
            
            // (2) Deposit 2 balls in high goal
            straight_PID(TILE - TRACKING_CENTER_DISTANCE_FROM_BACK, -TILE + TRACKING_CENTER_DISTANCE_FROM_BACK, true);
            turning_PID(135);
            straight_PID(0.4*TILE, -0.4*TILE, true);
            mechanism_state_var = command::low;

            // (3) Scraper
            turning_PID(135);
            straight_PID(2*TILE + 9.5, -2*TILE);
            turning_PID(180);

            // (5) Deposit in long goal
            straight_PID(2*TILE + 9.5, -3*TILE + 5); // scraper stuff, FALSE to going back
            pros::delay(20);
            straight_PID(2*TILE + 9.5, -TILE - 5, true);
            mechanism_state_var = command::high;
            
            break;
    }
    
    mechanism_state_var = command::stop;
}

// This is technically opcontrol, not autonomous, but since it's a helper function that's math-heavy, I'll leave it here

double exponential_drive(double input, double controllerDeadband, double drivetrainDeadband, double exponential_gain) // Default parameters are in the autonomous.hpp file
{
    double maximumEffectiveInput = 127 - controllerDeadband; // Need to do this so that maximum input maps to 1
    // shift input back by the deadband and then scale it so that it goes from 0 to 1
    double sign;

    if (input > 0) 
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    double normalizedInput = (fabs(input) - controllerDeadband)/maximumEffectiveInput; // This formula only works for positive numbers

    if (fabs(input) < controllerDeadband)
    {
        return 0;
    }

    return sign*(drivetrainDeadband + (127 - drivetrainDeadband)*powf(normalizedInput, exponential_gain));
}