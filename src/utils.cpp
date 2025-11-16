#include "robot/utils.hpp"
#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"
#include <sstream>
#include <cmath>

std::string formatDecimal(double value){
    std::ostringstream s;
    s.precision(2);
    s << std::fixed << value;
    return s.str();
}

// Corrección del parking
void correccion() {
    double targetHeading = 180.0;
    int duration = 1000;
    int startTime = pros::millis();

    while (pros::millis() - startTime < duration) {
        double currentHeading = fmod(chassis.getPose().theta, 360.0);
        if (currentHeading < 0) currentHeading += 360.0;

        if (currentHeading >= 160 && currentHeading <= 200) {
            robot_move(-80);
        } else {
            robot_move(0);
            chassis.turnToHeading(targetHeading, 1000, {.maxSpeed = 70});
        }
        pros::delay(20);
    }
    robot_move(0);
}


void correccionMach() {
    double targetHeading = 0;
    int duration = 1300;
    int startTime = pros::millis();

    while (pros::millis() - startTime < duration) {
        double currentHeading = fmod(chassis.getPose().theta, 360.0);
        if (currentHeading < 0) currentHeading += 360.0;

        if (currentHeading >= 340 || currentHeading <= 20) {
            robot_move(-70);
            if (chassis.getPose().y == 0) {
                robot_move(0); 
            }
        } else {
            robot_move(0);
            chassis.turnToHeading(targetHeading, 1000, {.maxSpeed = 70});
        }
        pros::delay(20);
    }
    robot_move(0);
}
