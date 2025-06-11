#include "init/diagonal_planner.h"
#include "init/extruder_servo.h"
#include "path_planning/CoordinatedMotion.h"

CoordinatedMotion motion(planner, extruderServo);