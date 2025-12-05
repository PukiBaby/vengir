#include "main.h"

#include <math.h>

#include "subsystems\declarations.hpp"
#include "subsystems\autonomous.hpp"
#include "subsystems\declarations.hpp"

void on_center_button() 
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) 
	{
		pros::lcd::set_text(2, "I was pressed!");
	} 
	else 
	{
		pros::lcd::clear_line(2);
	}
}

void initialize() 
{
	pros::lcd::initialize();
	imu.reset(true); // blocking = true
	pros::lcd::print(0, "vengir >:)");

	park.set_value(park_value); // Calibrate sensors

	// Odometry task

	pros::Task odometry_task(arc_odometry_fn, "odometry task"); 

	horizontalEnc.reset_position();
	verticalEnc.reset_position();

	// Odometry thread on screen

	pros::Task screenTask([&]() // 
	{
		while (true) 
		{
			// Print robot location to the brain screen

			pros::lcd::print(1, "X: %lf", pose_x); // x
			pros::lcd::print(2, "Y: %lf", pose_y); // y
			pros::lcd::print(3, "Theta (degrees): %lf", imu.get_heading()); // heading: degrees --> radians
			pros::lcd::print(4, "Odometry alive for: %d msec", odom_alive);
			
			// Delay to save resources
			pros::delay(50);
		}
	});
	
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() 
{
	autonomous_selection autonomous_variable = autonomous_selection::middle_control_middlestart;
	switch (autonomous_variable)    
    {
        case (autonomous_selection::test):
            pose_x = 0;
            pose_y = 0;
            break;

        case (autonomous_selection::middle_control_middlestart):
            pose_x = 0;
            pose_y = -3*TILE + PARK_ZONE_DEPTH + TRACKING_CENTER_DISTANCE_FROM_BACK + 4;
            break;

		case (autonomous_selection::left_high_goal_middlestart):
			pose_x = 0;
            pose_y = -3*TILE + PARK_ZONE_DEPTH + TRACKING_CENTER_DISTANCE_FROM_BACK + 4;
            break;

		case (autonomous_selection::right_high_goal_middlestart):
			pose_x = 0;
            pose_y = -3*TILE + PARK_ZONE_DEPTH + TRACKING_CENTER_DISTANCE_FROM_BACK + 4;
            break;
		
		case (autonomous_selection::awp_middle_start):
			pose_x = 0;
            pose_y = -3*TILE + PARK_ZONE_DEPTH + TRACKING_CENTER_DISTANCE_FROM_BACK + 4;
            break;
    }
	execute_autonomous(autonomous_variable);
}

void opcontrol()
{
	while (true) 
	{
    	// Get joystick positions

    	int straight = exponential_drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    	int turn = exponential_drive(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    	// Move the chassis with curvature drive

		left_mg.move(straight + turn);
		right_mg.move(straight - turn);

		// Hold

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			execute_command(command::collecting);
			pros::delay(10); // hold
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			execute_command(command::high);
			pros::delay(10); // hold
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			execute_command(command::middle);
			pros::delay(10); // hold
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake_mg.move(170);
		}

		else 
		{
			execute_command(command::stop);
		}

		// Toggle

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			park_value = !park_value;
			park.set_value(park_value);
			pros::delay(170); // toggle
			scraper_value != scraper_value;
			scraper.set_value(scraper_value);
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			scraper_value != scraper_value;
			scraper.set_value(scraper_value);
			pros::delay(170); // toggle
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			descore_value = !descore_value;
			descore.set_value(descore_value);
			pros::delay(170); // toggle
		}

		pros::delay(20);
	}
}