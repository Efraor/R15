#include "robot/motors.hpp"
#include "robot/utils.hpp"
#include "robot/chassis_config.hpp"
#include "robot/autonomous.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol1() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    bool estadoPiston1 = false;
    bool estadoPiston2 = false;
    bool estadoPiston3 = false;

    while (true) {

        // ------------- Manejo del chasis -------------
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        leftMotors.move(leftY);
        rightMotors.move(rightY);

        // Mostrar posición en pantalla
        lemlib::Pose pose = chassis.getPose();
        controller.set_text(
            0, 0,
            "X:" + formatDecimal(pose.x) +
            " Y:" + formatDecimal(pose.y) +
            " T:" + formatDecimal(pose.theta)
        );

        // ------------- Control de rollers / intake -------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
            roller.move(65);
            piston3Puerta.set_value(false);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            roller.move(-127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
            roller.brake();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);
            roller.move(127);
            piston3Puerta.set_value(true);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            intake.move(90);
            roller.move(60);
            piston3Puerta.set_value(true); 
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake.move(-80);
            roller.move(-60); 
        }
        else {
            intake.brake();
            roller.brake();
        }

        // ------------- Control de pistones -------------
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            estadoPiston1 = !estadoPiston1;
            piston1Brazo.set_value(estadoPiston1);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            estadoPiston2 = !estadoPiston2;
            piston2Loader.set_value(estadoPiston2);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            estadoPiston3 = !estadoPiston3;
            piston3Puerta.set_value(estadoPiston3);
        }

        // ------------- Ejecutar autónomos desde el control -------------
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            winpoint2();
        }
        
        // if (controller.get_digital_new_press(DIGITAL_UP)) {
        //     chassis.setPose(-52.334, 17.724, 90);
        // }

        if (controller.get_digital_new_press(DIGITAL_UP)) {
            chassis.setPose(-52.334, 17.724, 90);
        }
        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            chassis.moveToPose(0, 71, 0, 30004);
        }

        pros::delay(25);
    }
}
