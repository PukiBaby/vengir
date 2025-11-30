// HEADER GUARDS

#ifndef MECHANISMS_H
#define MECHANISMS_H

// OTHER LIBRARIES

#include "main.h"

#include <math.h>

#include "subsystems\declarations.hpp"

// FORWARD DECLARATIONS

enum class command : int {collecting, low, middle, high, stop};
void execute_command (command cmd);

#endif // MECHANISMS_H