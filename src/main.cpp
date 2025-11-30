#include "main.h"

#include <math.h>

#include "subsystems\declarations.hpp"
#include "subsystems\autonomous.hpp"
#include "subsystems\declarations.hpp"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() 
{
	pros::lcd::initialize();
	imu.reset();
	pros::lcd::print(0, "vengir >:)");

	park.set_value(park_value); // Calibrate sensors

	// Odometry task

	pros::Task odometry_task(arc_odometry_fn, "odometry task");

	// Odometry thread on screen

	pros::Task screenTask([&]() // 
	{
		while (true) 
		{
			// Print robot location to the brain screen

			pros::lcd::print(1, "X: %lf", pose_x); // x
			pros::lcd::print(2, "Y: %lf", pose_y); // y
			pros::lcd::print(3, "Theta (degrees): %lf", imu.get_heading()); // heading: degrees --> radians
			// pros::lcd::print(4, "Horizontal Encoder: %lf", horizontalEnc_raw);
			// pros::lcd::print(5, "Vertical Encoder: %lf", verticalEnc_raw);
			// pros::lcd::print(6, "Correction: %lf", correction);
			// pros::lcd::print(7, "odometry: %d", odometry_is_ready);
			
			// Delay to save resources
			pros::delay(50);
		}
	});

	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    
		int turn = master.get_analog(ANALOG_RIGHT_X);  
		left_mg.move(exponential_drive(dir + turn));                      
		right_mg.move(exponential_drive(dir - turn));                     
		pros::delay(20);                               
	}
}