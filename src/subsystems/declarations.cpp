#include "main.h"

#include "subsystems\autonomous.hpp"
#include "subsystems\mechanisms.hpp"
#include "subsystems\declarations.hpp"

#include <math.h>

// Controller

pros::Controller master (pros::E_CONTROLLER_MASTER);

// Driving

pros::MotorGroup right_mg ({DRIVING_RB_PORT, DRIVING_RM_PORT, DRIVING_RF_PORT});
pros::MotorGroup left_mg ({DRIVING_LB_PORT, DRIVING_LM_PORT, DRIVING_LF_PORT});

pros::MotorGroup everything_mg ({DRIVING_RB_PORT, DRIVING_RM_PORT, DRIVING_RF_PORT, DRIVING_LB_PORT, DRIVING_LM_PORT, DRIVING_LF_PORT}, pros::MotorGearset::blue, pros::v5::MotorEncoderUnits::rotations);

// Game mechanisms

pros::MotorGroup intake_mg ({MOTOR_UPTAKE_PORT, MOTOR_INTAKE_PORT});

// Pneumatics mechanisms

pros::adi::DigitalOut park (PARK_PORT); 
bool park_value = false;

pros::adi::DigitalOut scraper (SCRAPER_PORT);
bool scraper_value = false;

pros::adi::DigitalOut descore (DESCORE_PORT);
bool descore_value = true;

pros::adi::DigitalOut outtake_pneumatics (OUTTAKE_PORT);
bool outtake_value = true;

// Odometry

pros::Rotation horizontalEnc (H_ENC_PORT);
pros::Rotation verticalEnc (-V_ENC_PORT);
pros::IMU imu (IMU_PORT);