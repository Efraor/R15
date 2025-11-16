#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"
#include "robot/utils.hpp"
#include "lemlib/api.hpp"
#include "pros/apix.h"

// Si en algún momento usas paths de LemLib, puedes declarar tus assets aquí:
// ASSET(path1_txt);
// ASSET(chicoskills_txt);

// ---------------------- AUTONOMO 1 ----------------------
void autonomous1() {
    chassis.setPose(-46.403, 5.138, 0);
    chassis.moveToPoint(-46.40, 46.7, 2000, {.maxSpeed = 90, .minSpeed = 70} , false);
    piston2Loader.set_value(true);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);

    move_roller(127);

    chassis.moveToPoint(-60.5, 45.7, 3000, {.maxSpeed = 90, .minSpeed = 70} , false);
    pros::delay(1500);

    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2Loader.set_value(false);

    chassis.turnToHeading(135, 1000, {.maxSpeed = 60}, false);
    chassis.moveToPoint(-11.83, 10.52, 3000, {.maxSpeed = 70} , false);

    piston3Puerta.set_value(true);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();
}

// ---------------------- AUTONOMO 2 ----------------------
void autonomous2() {
    chassis.setPose(-46.403, 5.138, 0);
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70} , false);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);

    piston2Loader.set_value(true);
    move_roller(127);

    chassis.moveToPoint(-61, 45.7, 3000, {} , false);
    pros::delay(2100);

    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2Loader.set_value(false);

    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);
    move_roller(-127);
    pros::delay(850);
    No_move_roller();

    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    piston2Loader.set_value(true);
    chassis.moveToPoint(-59, 45.7, 3000, {} , false);
    move_roller(127);
    pros::delay(2100);
    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2Loader.set_value(false);

    chassis.turnToHeading(135, 1000, {.maxSpeed = 60}, false);
    piston3Puerta.set_value(true);
    chassis.moveToPose(-10.3, 8.6, -225, 3000, {.maxSpeed = 70} , false);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();
    piston3Puerta.set_value(false);
}

// ---------------------- AUTONOMO 4 ----------------------
void autonomous4() {
    // Establece la posición inicial del robot en el campo
    // (coordenadas X = -46.403, Y = 5.138, orientación = 0°)
    chassis.setPose(-46.403, 5.138, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //-------------------------------------------------
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70} , false);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);

    piston2Loader.set_value(true);
    move_roller(127);

    chassis.moveToPoint(-60, 45.7, 3000, {} , false);
    pros::delay(2100);

    No_move_roller();
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    piston2Loader.set_value(false);

    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);
    move_roller(-127);
    pros::delay(850);
    No_move_roller();

    // Código comentado original:
    // chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    // piston2Loader.set_value(true);
    // chassis.moveToPoint(-59, 45.7, 3000, {} , false);
    // move_roller(127);
    // pros::delay(2100);
    // No_move_roller();
    // chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false} , false);
    // piston2Loader.set_value(false);

    chassis.turnToHeading(135, 1500, {.maxSpeed = 60}, false);
    piston3Puerta.set_value(true);
    intake.move(100);

    chassis.moveToPose(-10.3, 8.6, -225, 3000, {.maxSpeed = 90} , false);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();
    piston3Puerta.set_value(false);
}

// ---------------------- AUTONOMO 3 ----------------------
void autonomous3() {
    // Establece la posición inicial del robot en el campo
    // (coordenadas X = -46.403, Y = 5.138, orientación = 0°)
    chassis.setPose(-46.403, 5.138, 0);

    // Configura el freno de los motores en modo “COAST”
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //-------------------------------------------------
    // Movimiento hacia el primer roller
    chassis.moveToPoint(-46.40, 45.4, 3500, {.maxSpeed = 70}, false);
    piston2Loader.set_value(true);

    chassis.turnToHeading(270, 1200, {.maxSpeed = 80, .minSpeed = 40}, false);

    move_roller(127);

    chassis.moveToPose(-65, 45.4, 270, 3000, {.maxSpeed = 80, .minSpeed = 60}, false);
    robot_move(70);
    pros::delay(2200);

    robot_move(0);
    No_move_roller();
    chassis.moveToPoint(-46.40, 45.4, 2000, {.forwards = false}, false);
    piston2Loader.set_value(false);

    // Roller inverso
    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);
    move_roller(-127);
    pros::delay(750);
    No_move_roller();

    chassis.turnToHeading(270, 1200, {.maxSpeed = 80, .minSpeed = 40}, false);
    piston2Loader.set_value(true);
    chassis.moveToPose(-65, 45.4, 270, 3000, {.maxSpeed = 80, .minSpeed = 60}, false);
    robot_move(70);

    pros::delay(1500);
    robot_move(0);

    chassis.moveToPoint(-46.40, 45.4, 2000, {.forwards = false}, false);


    // Movimiento hacia goal / intake largo
    chassis.turnToHeading(135, 1500, {.maxSpeed = 60}, false);
    piston3Puerta.set_value(true);
    intake.move(100);

    chassis.moveToPose(-10.3, 8.6, -225, 3000, {.maxSpeed = 90}, false);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();
    piston3Puerta.set_value(false);
}

void autonomous5() {
    // Establece la posición inicial del robot en el campo
    // (coordenadas X = -46.403, Y = 5.138, orientación = 0°)
    chassis.setPose(-51.43, 15, 0);

    // Configura el freno de los motores en modo “COAST”
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //-------------------------------------------------
    // Movimiento hacia el primer roller
    chassis.moveToPose(-51.43, 47.6, 0, 3500, {.maxSpeed = 70, .minSpeed = 50}, false);
    piston2Loader.set_value(true);

    chassis.turnToHeading(270, 1200, {.maxSpeed = 55, .minSpeed = 40}, false);
    pros::delay(200);

    move_roller(127);

    int loaderY = chassis.getPose().y;

for (int i = 0; i < 2; i++) {
    chassis.turnToHeading(270, 1200, {.maxSpeed = 55, .minSpeed = 40}, false);
    chassis.moveToPose(-66.5, loaderY, 270, 2000, {.maxSpeed = 100, .minSpeed = 80}, false);
    pros::delay(100);
}

    pros::delay(1500);

    No_move_roller();
    chassis.moveToPoint(-46.40, chassis.getPose().y, 2000, {.forwards = false}, false);
    piston2Loader.set_value(false);
    pros::delay(200);

    // Roller inverso
    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);
    move_roller(-127);
    pros::delay(1100);
    No_move_roller();

    chassis.turnToHeading(270, 1200, {.maxSpeed = 80, .minSpeed = 40}, false);
    piston2Loader.set_value(true);
    move_roller(127);
    for (int i = 0; i < 2; i++) {
    chassis.turnToHeading(270, 1200, {.maxSpeed = 55, .minSpeed = 40}, false);
    chassis.moveToPose(-66.5, loaderY, 270, 2000, {.maxSpeed = 100, .minSpeed = 80}, false);
    pros::delay(100);
}
    pros::delay(2000);
    No_move_roller();


    chassis.moveToPoint(-46.40, chassis.getPose().y, 2000, {.forwards = false}, false);
    piston2Loader.set_value(false);



    // Movimiento hacia goal / intake largo
    chassis.turnToHeading(135, 1500, {.maxSpeed = 60}, false);
    piston3Puerta.set_value(true);
    intake.move(100);

    chassis.moveToPose(-10.5, 7.63, -225, 3000, {.maxSpeed = 90}, false);
    move_roller(90);
    pros::delay(4500);
    No_move_roller();
    piston3Puerta.set_value(false);
}

void winPoint() {
    //Avanza al center
    chassis.setPose(-52.334, 17.724, 90);
    chassis.moveToPoint(-17.63, 17.65, 3000, {.maxSpeed = 40}, false);
    chassis.moveToPose(-10.75, 14.5, 139.6, 3000, {.maxSpeed = 60}, false);
    pros::delay(100);
    piston3Puerta.set_value(true);
    pros::delay(100);
    move_roller(90);
    pros::delay(2000);
    No_move_roller();

    //Se posiciona para el loader
    
    chassis.moveToPose(-49.86, 48.1, 137, 3000, {.forwards = false,.maxSpeed = 80}, false);
    piston3Puerta.set_value(false);
    piston2Loader.set_value(true);
    chassis.turnToHeading(270, 1500, {.maxSpeed = 60}, false);
    int loaderY = chassis.getPose().y;
    move_roller(90);
    // // for (int i = 0; i < 3; i++) {
    // // chassis.moveToPose(-60.52, loaderY, 270, 1000, {.minSpeed = 60}, false);
    // // pros::delay(1000);
    // // }
    robot_move(50);
    pros::delay(800);
    robot_move(2);
    pros::delay(3000);
    No_move_roller();
    piston2Loader.set_value(false);
    pros::delay(200);
    chassis.moveToPose(-40.61, 46.47, 270, 1500, {.forwards = false,.maxSpeed = 60}, false);
    chassis.turnToHeading(315, 1500, {.maxSpeed = 60}, false);

    pros::delay(100);
    move_roller(-90);
    pros::delay(1100);
    No_move_roller();

    //Se posiciona al long
    chassis.turnToHeading(90, 1500, {.maxSpeed = 60}, false);
    piston1Brazo.set_value(true);
    chassis.moveToPose(-25, 48.9, 90, 2100, {.maxSpeed = 40}, false);
    piston3Puerta.set_value(true);
    pros::delay(200);
    intake.move(127);
    roller.move(127);
    pros::delay(3000);
    No_move_roller();
    chassis.setPose(-25, 48.9, 90);
    chassis.moveToPose(-30, 48.9, 90, 3000, {.maxSpeed = 40}, false);
    chassis.turnToHeading(60, 1500, {.maxSpeed = 60}, false);
    chassis.moveToPose(-60, 25, 10, 3000, {.forwards = false,.maxSpeed = 60}, false);
    piston1Brazo.set_value(false);
    piston3Puerta.set_value(false);
    robot_move(-70);
    pros::delay(150);

    correccionMach();
    

}

// ---------------------- SKILLS 2 ----------------------
void skills2 () {
    //Inicio
    chassis.setPose(-50.44, 6.33, 90);

    //Casi medio
    chassis.moveToPose(-9.67, 16.21, 60, 3000, {.maxSpeed = 100}, false);
    move_roller(120);

    // Medio
    chassis.swingToHeading(20, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 80});
    //chassis.moveToPose(-3.62, 20.7, 0, 3000, {.maxSpeed = 60}, false);

    //Recoge
    chassis.moveToPose(-3.62, 39, 0, 4000, {.maxSpeed = 30}, false);
    No_move_roller();

    //Se posiciona para anotar
    chassis.moveToPose(-24, 23.1, 45, 3000, {.forwards = false, .maxSpeed = 60}, false);
    
    //Anota
    chassis.turnToHeading(130, 1000);
    chassis.moveToPose(-15., 15.41, 142, 5000, {.maxSpeed = 70}, false);
    piston3Puerta.set_value(true);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();

    //Se posiciona para el otro lado
    chassis.moveToPose(-19, 21, 120, 1000, {.forwards = false, .maxSpeed = 100}, false);
    piston3Puerta.set_value(false);

    //se va al otro lado
    chassis.moveToPose(-18.24, -13.74, 142, 3000, {.maxSpeed = 80}, false);
    chassis.moveToPose(-6.10, -19, 104, 3000, {.maxSpeed = 80}, false);
    move_roller(120);
    chassis.swingToHeading(148, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 120});

    //se pociciona para recoger
    chassis.moveToPose(0, -41.4, 180, 4000, {.maxSpeed = 30}, false);
    No_move_roller();
    chassis.turnToHeading(135, 1000);

    //Se posiciona para anotar
    chassis.moveToPose(-22.11, -25.75, 135, 3000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPose(-10, -8.57, 48.1, 3000, {.maxSpeed = 50}, false);
    move_roller(-80);
    pros::delay(4000);
    No_move_roller();

    // Movimiento final + corrección
    chassis.moveToPose(-54.3, -28.2, 160, 3000, {.forwards= false, .maxSpeed = 60}, false);
    chassis.moveToPose(-62.86, 0, 180, 2000, {.forwards= false, .minSpeed =127 }, false);
    correccion();
}

// ---------------------- SKILLS (VERSIÓN COMENTADA) ----------------------
void skills () {
    // --- Inicio de la rutina Skills ---
    chassis.setPose(-50.44, 6.33, 90);

    // --- Movimiento hacia casi medio ---
    chassis.moveToPose(-9.67, 16.21, 60, 3000, {.maxSpeed = 100}, false);
    move_roller(120);

    // --- Movimiento al centro ---
    chassis.swingToHeading(20, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 80});
    
    // --- Recoger bolas ---
    chassis.moveToPose(-3.62, 39, 0, 4000, {.maxSpeed = 30}, false);
    No_move_roller();

    // --- Posicionamiento para anotar ---
    chassis.moveToPose(-24, 23.1, 45, 3000, {.forwards = false, .maxSpeed = 60}, false);

    // --- Anotación en los goals ---
    chassis.turnToHeading(130, 1000);
    chassis.moveToPose(-15., 15.41, 142, 5000, {.maxSpeed = 70}, false);
    piston3Puerta.set_value(true);
    move_roller(127);
    pros::delay(3000);
    No_move_roller();

    // --- Preparación para volver a recoger ---
    chassis.moveToPose(-19, 21, 120, 1000, {.forwards = false, .maxSpeed = 100}, false);
    piston3Puerta.set_value(false);

    // --- Movimiento hacia el otro lado de la cancha ---
    chassis.moveToPose(-18.24, -13.74, 142, 3000, {.maxSpeed = 80}, false);
    chassis.moveToPose(-6.10, -19, 104, 3000, {.maxSpeed = 80}, false);
    move_roller(120);
    chassis.swingToHeading(148, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 120});

    // --- Posicionamiento para segunda recogida ---
    chassis.moveToPose(0, -41.4, 180, 4000, {.maxSpeed = 30}, false);
    No_move_roller();
    chassis.turnToHeading(135, 1000);

    // --- Posicionamiento para segunda anotación ---
    chassis.moveToPose(-22.11, -25.75, 135, 3000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPose(-10, -8.57, 48.1, 3000, {.maxSpeed = 50}, false);
    move_roller(-80);
    pros::delay(4000);
    No_move_roller();

    // --- Movimiento final hacia posición de parking ---
    chassis.moveToPose(-54.3, -28.2, 160, 3000, {.forwards= false, .maxSpeed = 60}, false);
    chassis.moveToPose(-62.86, 0, 180, 2000, {.forwards= false, .minSpeed =127 }, false);

    // --- Corrección de parking ---
    correccion();
}
