#include "main.h"

#include "subsystems\autonomous.hpp"
#include "subsystems\mechanisms.hpp"
#include "subsystems\declarations.hpp"

#include <math.h>

// Enum defined in header

void execute_command(command cmd) {
  switch (cmd) 
  {
    case command::collecting:
        if (pros::competition::is_autonomous())
        {
            park_value = false;
            park.set_value (park_value);
        }

        intake_mg.move(-127);

        break;

    case command::low:
        if (pros::competition::is_autonomous())
        {
            park_value = false;
            park.set_value (park_value);
        }

        intake_mg.move(127);

        break;

    case command::middle:
        if (pros::competition::is_autonomous())
        {
            park_value = false;
            park.set_value (park_value);

            outtake_value = false;
            outtake_pneumatics.set_value (outtake_value);
        }

        intake_mg.move(127);

        break;

    case command::high:
        if (pros::competition::is_autonomous())
        {
            park_value = false;
            park.set_value (park_value);

            outtake_value = true;
            outtake_pneumatics.set_value (outtake_value);
        }

        intake_mg.move(127);

        break;

    case command::stop:
        intake_mg.move(0);

        break;

    default:
        std::cout << "Error: Unhandled Robot Command detected!" << std::endl;
        
        break;
  }
}