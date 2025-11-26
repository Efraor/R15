#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"
#include "robot/utils.hpp"
#include "lemlib/api.hpp"
#include "pros/apix.h"


ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);
ASSET(skills4_txt);
ASSET(skills5_txt);
ASSET(PARKING_txt);

void winpoint2(){
    //Empieza
    chassis.setPose(-52.5, 17.11, 0);

    //Se mueve al loader
    chassis.moveToPose(-52.5, 47.85, 0, 3000, {.maxSpeed = 70}, false);
    piston2Loader.set_value(false);
    chassis.turnToHeading(273.5, 1500, {.maxSpeed = 70}, false);
    loaderLoad(2);
    
    //SALE DEL LOADER
    int loaderY = chassis.getPose().y;
    chassis.moveToPoint(-51, loaderY, 3000, {.forwards = false, .maxSpeed = 70}, false);
    chassis.moveToPoint(-51, loaderY, 3000, {.forwards = false, .maxSpeed = 80}, false);
    piston2Loader.set_value(true);
    chassis.turnToHeading(130, 1500, {.maxSpeed = 80}, false);
    move_roller(80);
    
    
    //VA AL CENTER GOAL
    chassis.moveToPose(-9.58, 12.66, 138, 3000, {.maxSpeed = 60}, false);
    piston3Puerta.set_value(true);
    robot_move(30);
    pros::delay(150);
    robot_move(0);
    move_roller(120);
    pros::delay(1400);
    move_roller(-60);
    piston3Puerta.set_value(false);
    move_roller(-60);
    robot_move(-60);
    pros::delay(200);
    move_roller(0);
    robot_move(0);
    
    //Se posiciona al otro
    chassis.turnToHeading(45, 1500, {.maxSpeed = 80}, false);
    piston3Puerta.set_value(false);
    chassis.moveToPose(-28.06, -23.5, 0, 3000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.turnToHeading(40, 1500, {.maxSpeed = 60}, false);
    chassis.moveToPose(-10, -8.2, 47.5, 1500, {.maxSpeed = 70}, false);
    move_roller(-110);
    pros::delay(8500);

    //Se va a estacionar
    chassis.follow(PARKING_txt,15,3000,false,false);
    correccionMach();    
}


void skillsp() {
    chassis.setPose(-52.334, 17.724, 90);
    move_roller(90);
    chassis.follow(skills1_txt, 15, 5500, true, false);
    move_roller(0);
    chassis.moveToPose(-1.879, 33.295, 0, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.turnToHeading(45, 1200);
    chassis.moveToPose(-22.37, 28, 139, 3000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.turnToHeading(130, 1200);
    chassis.moveToPose(-12, 16.78, 136, 5000, {.maxSpeed = 70}, false);
    piston3Puerta.set_value(true);
    move_roller(127);
    pros::delay(4000);
    No_move_roller();

    //Se posiciona para el otro lado
    chassis.moveToPose(-22.37, 28, 120, 3000, {.forwards = false, .maxSpeed = 60}, false);

    piston3Puerta.set_value(false);

    //se va al otro lado
    chassis.moveToPose(-18.24, -13.74, 142, 3000, {.maxSpeed = 80}, false);
    chassis.moveToPose(-6.10, -19, 104, 3000, {.maxSpeed = 80}, false);
    move_roller(120);
    chassis.swingToHeading(148, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 120});

    //se pociciona para recoger
    chassis.moveToPose(1.2, -41.4, 180, 4000, {.maxSpeed = 30}, false);
    chassis.moveToPose(1.2, -35.4, 180, 4000, {.maxSpeed = 60}, false);
    No_move_roller();
    chassis.turnToHeading(135, 1000);

    //Se posiciona para anotar
    chassis.moveToPose(-22.11, -25.75, 135, 3000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPose(-9, -5.57, 46.1, 3000, {.maxSpeed = 50}, false);
    robot_move(40);
    pros::delay(120);
    move_roller(-70);
    pros::delay(5000);
    No_move_roller();

    // Movimiento final + corrección
    chassis.moveToPose(-54.3, -28.2, 160, 3000, {.forwards= false, .maxSpeed = 60}, false);
    chassis.moveToPose(-62.86, 0, 180, 2000, {.forwards= false, .minSpeed =127 }, false);
    correccionMach();



    //chassis.turnToHeading(218, 1200);
    //piston3Puerta.set_value(false);


}
