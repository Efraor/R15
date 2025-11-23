#include "main.h"
#include "robot/chassis_config.hpp"
#include "robot/autonomous.hpp"
#include "robot/opcontrol.hpp"

void initialize() {
    chassis_setup();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Cambia este número para elegir qué autónomo se usará en competencia
    winpoint2();  
}

void opcontrol() {
    opcontrol1();
}
