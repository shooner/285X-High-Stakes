//10-19-2025

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#define RED_SIG 1
#define BLUE_SIG 2
#define VISION_PORT 2

pros::Motor onetwo_motor(19, pros::MotorGearset::green);
pros::Motor threefour_motor(17, pros::MotorGearset::green); 
pros::Motor five_motor(3, pros::MotorGearset::green); 
pros::Motor six_motor(2, pros::MotorGearset::green); 
int basket = 1; //1 is lower, 2 is upper
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::ADIPort scraper('A', pros::E_ADI_DIGITAL_OUT);
pros::Vision vision_sensor (VISION_PORT);

static bool detect_signature (pros:: Vision& vision, std::uint8_t sig_id, int min_w = 6, int min_h = 6) {
    pros::vision_object_s_t objs[1];
    int32_t copied = vision.read_by_sig(0, sig_id, 1, objs);
    (void) copied;
    const auto& obj = objs[0];
    if (obj.signature == VISION_OBJECT_ERR_SIG) return false;
    if (obj.signature != sig_id) return false;
    // Filter out noise by requiring minimum size 
    return obj.width >= min_w && obj.height >= min_h;
}

void toggleBasket(void* param){
    /*while(true){
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
    }*/
   while(true){
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            // Clear the controller line and give the controller a few ms to apply it,
            // then print a padded string so we always overwrite any previous text.
            controller.clear_line(0);
            pros::delay(5);

            if(basket==1){
                basket = 2;
                controller.print(0, 0, "%-13s", "Top Basket");      // pad to 13 chars
                pros::delay(100);
            }
            else if(basket==2){
                basket = 1;
                controller.print(0, 0, "%-13s", "Bottom Basket");   // longest message
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
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			// Color sorted intake
            static bool last_red = false;
            static bool last_blue = false;
            bool red_present = detect_signature (vision_sensor, RED_SIG);
            bool blue_present = detect_signature (vision_sensor, BLUE_SIG);
            if (red_present != last_red) {
                onetwo_motor.move(127);
				threefour_motor.move(127);
				five_motor.move(0);
				six_motor.move(-127);
                last_red = red_present;
            }
            if (blue_present != last_blue) {
                threefour_motor.move(127);
            	five_motor.move(-127);
				onetwo_motor.move(0);
				six_motor.move(0);
            } //Changes depending on our color
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