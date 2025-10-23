//10-19-2025


#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#define RED_SIG 1
#define BLUE_SIG 2
#define VISION_PORT 12


pros::Motor onetwo_motor(19, pros::MotorGearset::green);
pros::Motor threefour_motor(17, pros::MotorGearset::green);
pros::Motor five_motor(3, pros::MotorGearset::green);
pros::Motor six_motor(2, pros::MotorGearset::green);
int basket = 1; //1 is lower, 2 is upper
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::ADIPort scraper('A', pros::E_ADI_DIGITAL_OUT);
pros::Vision vision_sensor (VISION_PORT);
pros::MotorGroup left_motors({ 1, 1,1}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({1, 1, 1}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6  
pros::Rotation vertical(20);


lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13, // 12.5 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              400, // drivetrain rpm is 400
                              2 // horizontal drift is 2
);


lemlib::TrackingWheel vertical_wheel(&vertical, lemlib::Omniwheel::NEW_275, 0);
pros::Imu imu(10);


lemlib::OdomSensors sensors(&vertical_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);




lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angular_controller(8 , // proportional gain (kP)
                                              0, // integral gain (kI)
                                              69, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::ExpoDriveCurve throttle_curve(20, // joystick deadband out of 127
                                     20, // minimum output where drivetrain will move out of 127
                                     1.038 // expo curve gain
);




// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(20, // joystick deadband out of 127
                                  20, // minimum output where drivetrain will move out of 127
                                  1.048 // expo curve gain
);






lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);


static bool detect_signature (pros:: Vision& vision, std::uint8_t sig_id, int min_w = 6, int min_h = 6) {
    pros::vision_object_s_t objs[1];
    int32_t copied = vision.read_by_sig(0, sig_id, 1, objs);
    (void) copied;
    const auto& obj = objs[0];
    if (obj.signature == VISION_OBJECT_ERR_SIG) return false;
    if (obj.signature != sig_id) return false;
    // Filter out noise by requiring minimum size
    return (obj.width >= min_w && obj.height >= min_h);
}

void toggleBasket(void* param){
    while(true){
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            if(basket==1){
                basket = 2;
                controller.print(0, 0, "Top Basket");
                pros::delay(100);
            }
            else if(basket==2){
                basket = 1;
                controller.print(0, 0, "Bottom Basket");
                pros::delay(100);
            }
        }
        pros::delay(50);
    }
}

 void toggleScraper(void* param) {
    bool scraper_engaged = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            scraper_engaged = !scraper_engaged;  // Toggle scraper
            scraper.set_value(scraper_engaged);
            pros::delay(200);               // Prevent rapid toggling
        }

        pros::delay(20);
    }
}

void motorControl(void* param) {
    bool last_blue = false;
    bool last_red = false;
	while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            // Intake with color sorting
            bool blue_present = detect_signature(vision_sensor, BLUE_SIG);
            bool red_present = detect_signature(vision_sensor, RED_SIG);

        
            threefour_motor.move(127);  // Always intake
            if (red_present){
                last_red = true;
                last_blue = false;
            }

            if(blue_present){
                last_blue = true;
                last_red = false;
            }
            if(last_red){
                onetwo_motor.move(-127);
                five_motor.move(0);
                six_motor.move(-127);
            } if (last_blue){
                five_motor.move(-127);
                onetwo_motor.move(0);
                six_motor.move(0);
            } 
            pros::lcd::set_text(2, last_red ? "Red Detected" : "No Red");
            pros::lcd::set_text(3, last_blue ? "Blue Detected" : "No Blue");
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			// Outtake center lower
			if(basket==1){
				threefour_motor.move(-127);
				five_motor.move(127);
				onetwo_motor.move(0);
				six_motor.move(0);
			} else if(basket==2){
				six_motor.move(127);
				onetwo_motor.move(127);
				threefour_motor.move(-127);
				five_motor.move(0);
        	}
    	}
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			// Outtake center upper
            if(basket==1){
				five_motor.move(127);
				threefour_motor.move(127);
				onetwo_motor.move(127);
				six_motor.move(0);
			}
			else if(basket==2){
				six_motor.move(127);
				threefour_motor.move(127);
				onetwo_motor.move(127);
				five_motor.move(0);
			}
		}
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			// Outtake long goal
            if(basket==1){
                five_motor.move(127);
                threefour_motor.move(127);
                onetwo_motor.move(-127);
				six_motor.move(0);
            }
            else if(basket==2){
                six_motor.move(127);
                onetwo_motor.move(-127);
				threefour_motor.move(0);
				five_motor.move(0);
            }
        }
        else {
			// No button pressed - stop all motors
            six_motor.move(0);
			onetwo_motor.move(0);
			threefour_motor.move(0);
			five_motor.move(0);
        }
        pros::delay(20);
    }
}

void opcontrol(){
	pros::lcd::initialize();
    pros::vision_signature_s_t BLUE_SIGNATURE =
    pros::Vision::signature_from_utility (BLUE_SIG, -3461, -2881, -3172, 5123, 6215, 5668, 3.0, 0); 
    pros::vision_signature_s_t RED_SIGNATURE =
    pros::Vision::signature_from_utility(RED_SIG, 9843, 12289, 11066, -1681, -891, -1286, 3.0, 0);
    vision_sensor.set_signature (BLUE_SIG, &BLUE_SIGNATURE);
    vision_sensor.set_signature (RED_SIG, &RED_SIGNATURE);
	pros::Task basketTask (toggleBasket, NULL, "Basket Task");
	pros::Task scraperTask (toggleScraper, NULL, "Scraper Task");
	pros::Task motorControlTask (motorControl, NULL, "Motor Control Task");
	while(true){
		pros::delay(20);
	}

}

void autonomous() {

    int a = -1;
    int b = 1;


    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(a*65.25, b*13.198, 12);

    chassis.turnToPoint(a*59, b*35, 1000);
    chassis.moveToPoint(a*59, b*35, 1000);

    //intake 3 from drop loader

    chassis.turnToPoint(a*58.818, b*46.72, 1000);
    chassis.moveToPoint(a*58.818, b*46.72, 1000);
    chassis.turnToHeading(-90, 1000);
    scraper.set_value(true);   //engage scraper
    convAuton(1, 0);
    pros::delay(500);
    chassis.moveToPoint(a*66.88, 46.72, 1000);
    pros::delay(500);
    chassis.moveToPoint(a*58.818, b*46.72, 1000);
    scraper.set_value(false);  //disengage scraper

    chassis.turnToPoint(a*42.5, b*45.95, 1000);
    chassis.moveToPoint(a*42.5, b*45.95, 1000);

    chassis.turnToPoint(a*22.54, b*22.35, 1000);
    chassis.moveToPoint(a*22.54, b*22.35, 1000);

    chassis.turnToPoint(a*8.14, b*9.10, 1000);
    chassis.moveToPoint(a*8.14, b*9.10, 1000);

    //dump onto lower mid
    convAuton(1, 1);
    pros::delay(500);
    convOff();

    chassis.moveToPoint(a*22.54, b*22.35, 1000);

    chassis.turnToPoint(a*23.309, a*22.373, 1000);
    convAuton(1, 0);
    chassis.moveToPoint(a*23.309, a*22.373, 1000);
    convOff();

    chassis.turnToPoint(a*9.61, a*8.36, 1000);
    chassis.moveToPoint(a*9.61, a*8.36, 1000);

    convAuton(1, 2);
    pros::delay(500);
    convOff();




    while (true){
        pros::delay(10);
    }
}


 //end of q2 bonus route

//start of q3 bonus route
