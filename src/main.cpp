#include "main.h"
#include "lemlib/api.hpp"


// Control
pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::string formatDecimal(double value) {
    std::ostringstream stream;
    stream.precision(2);     // 2 decimales
    stream << std::fixed << value;
    return stream.str();
}

// Grupo de motores chassis
pros::MotorGroup leftMotors({-18, -19},pros::MotorGearset::green); // Grupo de motores izquierdo - puerto 11 (reversa), 14(reversa)
pros::MotorGroup rightMotors({16, 15}, pros::MotorGearset::green); // Grupo de motores derecgos - puerto 12 y 13

// Motores para los rollers
pros::Motor intake(-20, pros::MotorGearset::green); //  puerto 15
pros::Motor roller(1, pros::MotorGearset::green); //  puerto 16
// pros::Motor roller3(-1, pros::MotorGearset::green); //  puerto 17

pros::adi::DigitalOut piston1Brazo('G');   // Piston puerto "C" PUERTA
pros::adi::DigitalOut piston2Loader('H');   // Piston puerto "C"H
pros::adi::DigitalOut piston3Puerta('F');   // Piston puerto "C"G LOADER F

//Sensor inercial
pros::Imu imu(17);  //Puerto 17




// Esto no usamos por el momento pero es para usar los tracking wheels


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// Configuracion del drivetrain 
lemlib::Drivetrain drivetrain(&leftMotors,  // Grupo de motores izquierdo
                              &rightMotors, // Grupo de motores derecho
                              12.5,         // 12.5 inch track width(la distancia entre las líneas centrales de dos ruedas en el mismo eje en pulgadas)
                              lemlib::Omniwheel::NEW_325, // rueda omni de 4
                              466.66,       // drivetrain rpm is 333.33
                              2             // la deriva horizontal es 2. Si tuviéramos ruedas de tracción, habría sido 8
);

// controlador de movimiento lineal
lemlib::ControllerSettings linearController(10,     // ganancia proporcional (kP)
                                            0,      // ganancia integral (kI)
                                            3,      // ganancia derivativa (kD)
                                            3,      // anti windup
                                            1,      // rango de error pequeño, en pulgadas
                                            100,    // tiempo de espera para rango de error pequeño, en milisegundos
                                            3,      // rango de error grande, en pulgadas
                                            500,    // tiempo de espera para rango de error grande, en milisegundos
                                            60      // aceleración máxima (giro)
);

// Controlador de movimiento angular
lemlib::ControllerSettings angularController(4,     // ganancia proporcional (kP)
                                             0,     // ganancia integral (kI)
                                             40,    // ganancia derivativa (kD)
                                             3,     // anti windup
                                             1,     // rango de error pequeño, en grados
                                             100,   // tiempo de espera para rango de error pequeño, en milisegundos
                                             3,     // rango de error grande, en grados
                                             500,   // tiempo de espera para rango de error grande, en milisegundos
                                             0      // aceleración máxima (giro)
);

// sensores para la odometria
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, Establecida en nullptr ya que no tenemos una segunda
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, Establecida en nullptr ya que no tenemos una segunda
                            &imu    // Sensor inercial
);

// curva de entrada para la entrada del acelerador durante el control del conductor
lemlib::ExpoDriveCurve throttleCurve(1, // Banda muerta del joystick de 127
                                     2, // salida mínima donde la transmisión se moverá fuera de 127
                                     1 // ganancia de la curva
);

// curva de entrada para la entrada de dirección durante el control del conductor
lemlib::ExpoDriveCurve steerCurve(1, // Banda muerta del joystick de 127
                                  2, // salida mínima donde la transmisión se moverá fuera de 127
                                  1 // ganancia de la curva
);

// creamios el chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/*
*   Ejecuta el código de inicialización. Esto ocurre en cuanto se inicia el programa.
*   Todos los demás modos de competición están bloqueados por la inicialización; se recomienda
*   mantener el tiempo de ejecución de este modo por debajo de unos pocos segundos.
*/
void initialize() {
    pros::lcd::initialize(); // Inicia la pantalladel cerebro
    imu.reset(); // inicia la calibración del IMU
    pros::lcd::print(0, "Calibrando IMU...");
    chassis.calibrate(); // Calibra sensores  
    // Esperar hasta que termine de calibrarse (tarda unos segundos)
    while (imu.is_calibrating()) {
        pros::delay(100); // esperar un poco para no saturar la CPU
    }
    pros::lcd::print(1, "IMU calibrado!");
    piston1Brazo.set_value(false);
    piston2Loader.set_value(false);
    piston3Puerta.set_value(false);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread para mostrar información en la pantalla del controlador principal y guardar datos de posición.
    pros::Task screenTask([&]() {
        while (true) {
            // imprime la ubicación del robot en la pantalla del cerebro
            pros::lcd::print(2, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(3, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(4, "Theta: %f", chassis.getPose().theta); // heading
            // telemetría de posición del registro
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay para ahorrar recursos
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(path1_txt); // '.' replaced with "_" to make c++ happy
ASSET(chicoskills_txt); // '.' replaced with "_" to make c++ happy

void move_roller (int number) {
    intake.move(number);
    roller.move(number);
    //roller3.move(127);
}

void No_move_roller () {
    intake.move(0);
    roller.move(0);
    //roller3.move(0);
}


void robot_move (int number) {
    leftMotors.move(number);
    rightMotors.move(number);
}




/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void autonomous() {
    chassis.setPose(-46.403, 5.138, 0);
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70} , false);
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    piston2Loader.set_value(true);
    move_roller(127);
    chassis.moveToPoint(-59, 45.7, 3000, {} , false);
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
    //chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
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

void correccion() {
double targetHeading = 180.0; // rumbo deseado
int duration = 1000;          // duración en milisegundos (2 segundos)
int startTime = pros::millis();

while (pros::millis() - startTime < duration) {
    double currentHeading = chassis.getPose().theta;

    // Normalizar heading a 0–360
    currentHeading = fmod(currentHeading, 360.0);
    if (currentHeading < 0) currentHeading += 360.0;

    // Si está dentro del rango, sigue moviéndose hacia atrás
    if (currentHeading >= 160 && currentHeading <= 200) {
        robot_move(-80); // mueve hacia atrás
    } 
    // Si se sale del rango, corrige con LemLib
    else {
        robot_move(0); // detiene para corregir
        chassis.turnToHeading(targetHeading, 1000, {.maxSpeed = 70});
    }

    pros::delay(20); // pequeña pausa para no saturar el CPU
}

// Detener al terminar el tiempo
robot_move(0);

}

void autonomous3() {
    // Establece la posición inicial del robot en el campo
    // (coordenadas X = -46.403, Y = 5.138, orientación = 0°)
    chassis.setPose(-46.403, 5.138, 0);

    // Configura el freno de los motores en modo “COAST” (ruedas giran libremente al soltar el motor)
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //-------------------------------------------------
    // Mueve el robot hacia adelante hasta el punto (-46.40, 45.7)
    // con una velocidad máxima del 70% y tiempo límite de 2 segundos
    chassis.moveToPoint(-46.40, 45.7, 2000, {.maxSpeed = 70}, false);

    // Gira el robot hasta el ángulo de 270° (mirando hacia la izquierda)
    // con velocidad máxima del 60% y tiempo límite de 1 segundo
    chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);

    // Activa el pistón 2 (Para bajar el mecanismo del roller)
    piston2Loader.set_value(true);

    // Enciende el rodillo (roller) en sentido positivo (velocidad máxima)
    move_roller(127);

    // Avanza un poco más hacia la izquierda para mantener contacto con el roller
    chassis.moveToPoint(-61, 45.7, 3000, {}, false);

    // Espera 2.1 segundos para completar el giro del roller
    pros::delay(2100);

    // Detiene el rodillo
    No_move_roller();

    // Retrocede al punto original (hacia atrás)
    chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false}, false);

    // Desactiva el pistón 2 (sube el mecanismo)
    piston2Loader.set_value(false);

    // Gira hacia la diagonal superior izquierda (315°)
    chassis.turnToHeading(315, 1000, {.maxSpeed = 60}, false);

    // Activa el rodillo en sentido contrario (para sacar o limpiar)
    move_roller(-127);

    // Espera 0.85 segundos para completar la acción
    pros::delay(850);

    // Detiene el rodillo nuevamente
    No_move_roller();

    // --- Código comentado (posible segunda repetición del roller) ---
    // chassis.turnToHeading(270, 1000, {.maxSpeed = 60}, false);
    // piston2Loader.set_value(true);
    // chassis.moveToPoint(-59, 45.7, 3000, {}, false);
    // move_roller(127);
    // pros::delay(2100);
    // No_move_roller();
    // chassis.moveToPoint(-46.40, 45.7, 2000, {.forwards = false}, false);
    // piston2Loader.set_value(false);
    // ---------------------------------------------------------------

    // Gira hacia 135° (diagonal inferior izquierda)
    chassis.turnToHeading(135, 1500, {.maxSpeed = 60}, false);

    // Activa el pistón 3 (probablemente para sujetar un disco o abrir la entrada del intake)
    piston3Puerta.set_value(true);

    // Enciende los motores del intake para absorber discos
    intake.move(100);

    // Mueve el robot hacia el punto (-10.3, 8.6) con orientación -225°
    // (movimiento largo hacia la zona contraria del campo)
    chassis.moveToPose(-10.3, 8.6, -225, 3000, {.maxSpeed = 90}, false);

    // Activa el roller nuevamente (posiblemente otro roller en el campo)
    move_roller(127);

    // Espera 3 segundos mientras el roller gira
    pros::delay(3000);

    // Detiene el roller
    No_move_roller();

    // Desactiva el pistón 3
    piston3Puerta.set_value(false);
}

void skills () {
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
    chassis.moveToPose(-54.3, -28.2, 160, 3000, {.forwards= false, .maxSpeed = 60}, false);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPose(-60.95, -26.58, 155, 3000, {.forwards= false, .maxSpeed = 60}, false);
    // chassis.moveToPose(-62.31, -16.91, 180, 3000, {.forwards= false, .maxSpeed = 60}, false);
    chassis.moveToPose(-62.86, 0, 180, 2000, {.forwards= false, .minSpeed =127 }, false);
    correccion();

}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); 
    bool estadoPiston1 = false;
    bool estadoPiston2 = false;
    bool estadoPiston3 = false;
    // Control
    // bucle para actualizar continuamente los motores
    while (true) {
        // obtener posiciones del joystick
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        leftMotors.move(leftY);
        rightMotors.move(rightY);
        // Mover el robot en modo tanque
        //chassis.tank(leftY, rightY);
        lemlib::Pose pose = chassis.getPose();
        std::string xStr = "X:" + formatDecimal(pose.x);
        std::string yStr = " Y:" + formatDecimal(pose.y);
        std::string tStr = " θ:" + formatDecimal(pose.theta);
        controller.set_text(0, 0, xStr + yStr + tStr);
        // Gira los rollers todos hacia adelante
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
            roller.move(65);
            piston3Puerta.set_value(false);
            //roller3.move(127);
        }
        // Gira los rollers todos hacia atras
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            roller.move(-127);
        }
        // Solo intake hacia adelante
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
            roller.brake();
        }
        // Solo un motor hacia adelante
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);
            roller.move(127);
            piston3Puerta.set_value(true);
        }
        // Frenar todos
        else {
            intake.brake();
            roller.brake();
        }
        
            // Control del pistón
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        estadoPiston1 = !estadoPiston1;         // Cambia de true a false
        piston1Brazo.set_value(estadoPiston1);

           // chassis.turnToHeading(0, 30000);
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        estadoPiston2 = !estadoPiston2;
        piston2Loader.set_value(estadoPiston2);
            //chassis.turnToHeading(90, 30000);
         }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        estadoPiston3 = !estadoPiston3;
        piston3Puerta.set_value(estadoPiston3);
            //chassis.turnToHeading(180, 30000);
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            autonomous3();
        }
        if (controller.get_digital_new_press(DIGITAL_B)) {
            skills();
        }
        // delay para ahorrar recursos
        pros::delay(25);
    }
}