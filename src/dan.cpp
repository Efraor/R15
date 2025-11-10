// #include "main.h"
// #include "lemlib/api.hpp"

// ASSET(park_txt); // Path file which contains the coordinates

// // Drivetrain Configuration
// pros::MotorGroup left_mg({ -9, -10, 20 }, pros::MotorGearset::green); // Left motor group
// pros::MotorGroup right_mg({ 1, 11, -12 }, pros::MotorGearset::green); // Right motor group
// // Drivetrain settings
// lemlib::Drivetrain drivetrain(&left_mg,					// left motor group
// 							  &right_mg,				// right motor group
// 							  13,						// 12.5 inch track width
// 							  lemlib::Omniwheel::NEW_4, // using old 4" omnis
// 							  466.666,					// drivetrain rpm is 466.666
// 							  2);						// horizontal drift is 2
// // IMU (Inertial Sensor)
// pros::Imu imu(5);
// // Optical shaft encoders
// pros::adi::Encoder adi_ver_encoder('E', 'F', true); // Vertical Encoder
// pros::adi::Encoder adi_hor_encoder('G', 'H'); // Horizontal Encoder
// // Vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&adi_ver_encoder, lemlib::Omniwheel::OLD_275, 5.5);
// // Horizontal tracking wheel
// lemlib::TrackingWheel horizontal_tracking_wheel(&adi_hor_encoder, lemlib::Omniwheel::OLD_275, -2);

// // Lemlib configuration
// lemlib::OdomSensors sensors(nullptr,	// vertical tracking wheel 1, set to null
// 							nullptr,					// vertical tracking wheel 2, set to nullptr
// 							nullptr, // horizontal tracking wheel 1
// 							nullptr,					// horizontal tracking wheel 2, set to nullptr
// 							&imu						// inertial sensor
// );
// // Lateral PID controller
// lemlib::ControllerSettings lateral_controller(5,  // proportional gain (kP)
// 											  0,   // integral gain (kI)
// 											  25,  // derivative gain (kD)
// 											  3,   // anti windup
// 											  1,   // small error range, in inches
// 											  100, // small error range timeout, in milliseconds
// 											  3,   // large error range, in inches
// 											  500, // large error range timeout, in milliseconds
// 											  20   // maximum acceleration (slew)
// );

// // Angular PID controller
// lemlib::ControllerSettings angular_controller(5,   // proportional gain (kP)
// 											  0,   // integral gain (kI)
// 											  52,  // derivative gain (kD)
// 											  3,   // anti windup
// 											  1,   // small error range, in degrees
// 											  100, // small error range timeout, in milliseconds
// 											  3,   // large error range, in degrees
// 											  500, // large error range timeout, in milliseconds
// 											  0	   // maximum acceleration (slew)
// );
// /**
//  * Runs initialization code. This occurs as soon as the program is started.
//  *
//  * All other competition modes are blocked by initialize; it is recommended
//  * to keep execution time for this mode under a few seconds.
//  */

//  // Chassis configuration
// lemlib::Chassis chassis(drivetrain,			// drivetrain settings
// 						lateral_controller, // lateral PID settings
// 						angular_controller, // angular PID settings
// 						sensors				// odometry sensors
// );

// // Robot's controller
// pros::Controller controller(pros::E_CONTROLLER_MASTER);

// pros::Vision visionSensor(4); // Vision Sensor

// // Superior Structure
// pros::adi::Pneumatics platform('c', false);		 // Platform Piston
// pros::MotorGroup intake({ -18, -8 }, pros::MotorGearset::green); // Left motor group
// pros::Motor outtake(19, pros::MotorGearset::green);

// // Initialization function
// void initialize()
// {
// 	chassis.calibrate(); // Calibrate the chassis sensors. DO NOT REMOVE
// 	/**
// 	 * The following Vision Signatures represent unique identifiers used for color-sorting detection.
// 	 * NOTE: For the Vision Sensor to reliably distinguish both colors, the ambient illumination
// 	 * must remain CLEAR and UNOBSTRUCTED.
// 	 */
// 	pros::vision_signature_s_t BLUE_BLOCK = pros::Vision::signature_from_utility(1, -3941, -3413, -3678, 6051, 8465, 7258, 3, 0);
// 	pros::vision_signature_s_t RED_BLOCK = pros::Vision::signature_from_utility(2, 6819, 8795, 7806, -861, -423, -642, 3, 0);

// 	// Sets the Signature to the Vision Sensor Memory
// 	visionSensor.set_signature(1, &BLUE_BLOCK);
// 	visionSensor.set_signature(2, &RED_BLOCK);
// }

// /**
//  * Runs while the robot is in the disabled state of Field Management System or
//  * the VEX Competition Switch, following either autonomous or opcontrol. When
//  * the robot is enabled, this task will exit.
//  */
// void disabled() {}

// /**
//  * Runs after initialize(), and before autonomous when connected to the Field
//  * Management System or the VEX Competition Switch. This is intended for
//  * competition-specific initialization routines, such as an autonomous selector
//  * on the LCD.
//  *
//  * This task will exit when the robot is enabled and autonomous or opcontrol
//  * starts.
//  */
// void competition_initialize() {}

// /**
//  * Runs the user autonomous code. This function will be started in its own task
//  * with the default priority and stack size whenever the robot is enabled via
//  * the Field Management System or the VEX Competition Switch in the autonomous
//  * mode. Alternatively, this function may be called in initialize or opcontrol
//  * for non-competition testing purposes.
//  *
//  * If the robot is disabled or communications is lost, the autonomous task
//  * will be stopped. Re-enabling the robot will restart the task, not re-start it
//  * from where it left off.
//  */

// void moveRobot(int voltage) {
// 	left_mg.move(voltage); // Move left side
// 	right_mg.move(voltage); // Move right side
// }

// void stopRobot() {
// 	left_mg.move(0); // Stop left side
// 	right_mg.move(0); // Stop right side
// 	pros::delay(50);
// }

// int blue = 1; // BLUE color match ID
// int red = 2; // RED color match ID

// /**
// * NOTE: Always set this flag to TRUE before competition
// */

// void outtakeBlocks() {
// 	intake.move(127);
// 	outtake.move(127);
// 	pros::delay(3500);
// 	intake.move(0);
// 	outtake.move(0);
// }

// // Intake / Outtake function (Color Sorting)
// void loadUntil(int color) {
// 	int startTime = pros::millis(); // Record the start time (ms)
// 	while (true) {
// 		intake.move(127);
// 		outtake.move(127);
// 		auto obj = visionSensor.get_by_sig(0, color);
// 		// Exit if object detected
// 		if (obj.signature == color && obj.width > (color == blue ? 205 : 145)) {
// 			intake.move(0);
// 			outtake.move(0);
// 			break;
// 		}
// 		// Exit if timeout (3.5s = 3500ms)
// 		if (pros::millis() - startTime > 3500) {
// 			intake.move(0);
// 			outtake.move(0);
// 			break;
// 		}
// 		pros::delay(10); // delay to save resources
// 	}
// }


// void autonomous()
// {
// 	// START
// 	chassis.setPose(0, 0, 0); // Set Zero Position
// 	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // Set chassis to COAST
// 	// Max Speed
// 	int turnMaxSpeed = 80;
// 	// Start from Parking Zone
// 	chassis.moveToPoint(0, 30.7, 5000, {}, false);
// 	stopRobot();
// 	// ------------------
// 	// -- FIRST MOVE -- 
// 	// ------------------
// 	// Turn to face the Block Dispenser
// 	chassis.turnToHeading(90, 1500, { .maxSpeed = turnMaxSpeed }, false);
// 	chassis.moveToPoint(5, chassis.getPose().y, 1000, { .maxSpeed = 80 , .minSpeed = 30 }, false);
// 	// Intake Until RED found
// 	loadUntil(red);
// 	// Move back to Long Goals
// 	chassis.moveToPose(-22, 34, 90, 5000, { .forwards = false, .minSpeed = 60 }, false);
// 	// Align to the Long Goal
// 	moveRobot(-30);
// 	pros::delay(600);
// 	stopRobot();
// 	// Outtake Until BLUE found
// 	loadUntil(blue);
// 	// ------------------
// 	// -- SECOND / THIRD MOVE -- 
// 	// ------------------
// 	for (int i = 0; i < 2; i++) {
// 		// Go back to dispenser
// 		chassis.moveToPose(2, 34.5, 90, 5000, { .minSpeed = 60 }, false);
// 		moveRobot(30);
// 		pros::delay(650);
// 		stopRobot();
// 		// Intake Until RED found
// 		loadUntil(red);
// 		// Move back to Long Goals
// 		chassis.moveToPose(-22, 34.5, 90, 5000, { .forwards = false, .minSpeed = 60 }, false);
// 		// Align to the Long Goal
// 		moveRobot(-30);
// 		pros::delay(700);
// 		stopRobot();
// 		outtakeBlocks();
// 	}
// }


// /**
//  * Runs the operator control code. This function will be started in its own task
//  * with the default priority and stack size whenever the robot is enabled via
//  * the Field Management System or the VEX Competition Switch in the operator
//  * control mode.
//  *
//  * If no competition control is connected, this function will run immediately
//  * following initialize().
//  *
//  * If the robot is disabled or communications is lost, the
//  * operator control task will be stopped. Re-enabling the robot will restart the
//  * task, not resume it from where it left off.
//  */

// void opcontrol()
// {
// 	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // Set Chassis to brake

// 	// Screen Decoration
// 	pros::screen::set_pen(pros::Color::red);
// 	pros::delay(10);
// 	pros::screen::set_eraser(pros::Color::red);
// 	pros::delay(10);
// 	pros::screen::fill_rect(0, 0, 480, 256);
// 	pros::delay(10);
// 	pros::screen::set_pen(pros::Color::ghost_white);
// 	pros::delay(10);
// 	pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "SPARK PARAGUAY");
// 	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "ROBOT");

// 	// Controller Decoration
// 	controller.print(0, 1, "Driver: MAXI"); // Current Driver

// 	// Jordan Flags
// 	bool piston = false;	 // Is the back piston extended / retracted
// 	int jordanMode = 0; // Activates Jordan Mode for wall stakes

// 	// Driver control
// 	while (true)
// 	{
// 		// Show Sensor values
// 		pros::screen::print(pros::E_TEXT_SMALL, 7, "X: %f, Y: %f, Theta: %f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
// 		pros::screen::print(pros::E_TEXT_MEDIUM, 11, "IMU: %f", imu.get_heading());

// 		// ARCADE control scheme
// 		int dir = controller.get_analog(ANALOG_LEFT_Y);			   // Gets amount forward/backward from left joystick
// 		double turn = controller.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick

// 		// Drivetrain (ARCADE MODE)
// 		left_mg.move(dir + turn);  // Sets left motor voltage
// 		right_mg.move(dir - turn); // Sets right motor voltage

// 		// Activate autonomous
// 		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
// 			autonomous();
// 		}
// 		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
// 			chassis.setPose(0, 0, 0);
// 			chassis.moveToPoint(0, 20, 500000, {}, false);
// 		}
// 		// Intake / Outtake control
// 		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
// 			// Intake forward
// 			intake.move(127);
// 			outtake.move(127);
// 		}
// 		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
// 			// Intake reverse
// 			intake.move(-127);
// 			outtake.move(-127);
// 		}
// 		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
// 			// Outtake forward
// 			intake.move(127);
// 			outtake.move(127);
// 		}
// 		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
// 			// Outtake Reverse
// 			intake.move(0);
// 			outtake.move(-127);
// 		}
// 		else {
// 			// Stop both
// 			intake.move(0);
// 			outtake.move(0);
// 		}

// 		// Platform Piston
// 		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
// 			platform.extend();
// 		}
// 		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
// 			platform.retract();
// 		}

// 		pros::delay(10); // delay to save resources. DO NOT REMOVE

// 	}
// }