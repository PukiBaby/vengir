#include "main.h"

#include <math.h>

#include "subsystems\declarations.hpp"

double exponential_drive(double input, double controllerDeadband = 3, double drivetrainDeadband = 10, double exponential_gain = 1.019)
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

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

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