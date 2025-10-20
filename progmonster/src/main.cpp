//10-19-2025

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
pros::Motor onetwo_motor(19, pros::MotorGearset::green); // left motors on ports 1, 2, 3
pros::Motor threefour_motor(17, pros::MotorGearset::green); // right motors on ports 4, 5, 6
pros::Motor five_motor(3, pros::MotorGearset::green); // left motors on ports 1, 2, 3
pros::Motor six_motor(2, pros::MotorGearset::green); // right motors on ports 4, 5, 6
int basket = 1; //1 is lower, 2 is upper
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::ADIPort scraper('A', pros::E_ADI_DIGITAL_OUT);


void toggleBasket(void* param){
    while(true){
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			controller.clear_line(0);
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
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            scraper_engaged = !scraper_engaged;  // Toggle scraper
            scraper.set_value(scraper_engaged);
            pros::delay(200);               // Prevent rapid toggling
        }

        pros::delay(20);
    }
}

void motorControl(void* param) {
	while (true) {
        // Check all buttons and control motors based on priority
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			// Intake
			if(basket==2){
				onetwo_motor.move(127);
				threefour_motor.move(127);
				five_motor.move(0);
				six_motor.move(-127);
			} else if (basket==1){
				threefour_motor.move(127);
            	five_motor.move(-127);
				onetwo_motor.move(0);
				six_motor.move(0);
			}
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			// Outtake center lower
			if(basket==1){
				threefour_motor.move(-127);
				five_motor.move(127);
				onetwo_motor.move(0);
				six_motor.move(0);
			} else if(basket==2){
				six_motor.move(127);
				onetwo_motor.move(-127);
				threefour_motor.move(-127);
				five_motor.move(-127);
        	}
    	}
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			// Outtake center upper
            if(basket==1){
				five_motor.move(127);
				threefour_motor.move(127);
				onetwo_motor.move(-127);
				six_motor.move(0);
			}
			else if(basket==2){
				six_motor.move(127);
				threefour_motor.move(127);
				onetwo_motor.move(-127);
				five_motor.move(0);
			}
		}
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			// Outtake long goal
            if(basket==1){
                five_motor.move(127);
                threefour_motor.move(127);
                onetwo_motor.move(127);
				six_motor.move(0);
            }
            else if(basket==2){
                six_motor.move(127);
                onetwo_motor.move(127);
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
	pros::Task basketTask (toggleBasket, NULL, "Basket Task");
	pros::Task scraperTask (toggleScraper, NULL, "Scraper Task");
	pros::Task motorControlTask (motorControl, NULL, "Motor Control Task");
	while(true){
		pros::delay(20);
	}

}