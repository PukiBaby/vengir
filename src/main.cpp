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
	autonomous_selection autonomous_variable = autonomous_selection::test;
	switch (autonomous_variable)    
    {
        case (autonomous_selection::test):
            pose_x = 0;
            pose_y = 0;
            break;

        case (autonomous_selection::middle_control): // need to adjust timings
            pose_x = 0;
            pose_y = -3*TILE + PARK_ZONE_DEPTH + TRACKING_CENTER_DISTANCE_FROM_BACK;
            break;
    }
	execute_autonomous(autonomous_variable);
}

void opcontrol() 
{
	while (true) 
	{
		int dir = master.get_analog(ANALOG_LEFT_Y);    
		int turn = master.get_analog(ANALOG_RIGHT_X);  
		left_mg.move(exponential_drive(dir + turn));                      
		right_mg.move(exponential_drive(dir - turn));                     
		pros::delay(20);                               
	}
}