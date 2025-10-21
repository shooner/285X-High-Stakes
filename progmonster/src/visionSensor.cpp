#include "main.h"
// Vision sensor configuration 
#define RED_SIG 1
#define BLUE_SIG 2
#define VISION_PORT 2

//Basic UI callback for testing purposes
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}
// Detect if a signature is present by checking object dimensions
// Uses read_by_sig to avoid stale cached results from get_by_sig
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


void initialize() {
// Setup LCD display with initial status
pros::lcd::initialize();
pros::lcd::set_text(1, "Vision Color Detection");
pros::lcd::set_text(2, "No Red object");
pros::lcd::set_text(3, "No Blue object");
pros::lcd::register_btn1_cb(on_center_button);
// Configure vision sensor with signatures from VEX Vision Utility 
pros::Vision vision_sensor (VISION_PORT);
pros::vision_signature_s_t BLUE_SIGNATURE =
pros::Vision::signature_from_utility (BLUE_SIG, -3461, -2881, -3172, 5123, 6215, 5668, 3.0, 0); 
pros::vision_signature_s_t RED_SIGNATURE =
pros::Vision::signature_from_utility(RED_SIG, 9843, 12289, 11066, -1681, -891, -1286, 3.0, 0);
vision_sensor.set_signature (BLUE_SIG, &BLUE_SIGNATURE);
vision_sensor.set_signature (RED_SIG, &RED_SIGNATURE);
}

void opcontrol() {
    pros::Controller master (pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({1, -2, 3});
    pros::MotorGroup right_mg({-4, 5, -6}); pros::Vision vision_sensor (VISION_PORT);
    while (true) {
        // Display LCD button states
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2, 
            (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
            (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
        // Track previous detection state to only update LCD when status changes 
        static bool last_red = false;
        static bool last_blue = false;
        // Check for color signatures
        bool red_present = detect_signature (vision_sensor, RED_SIG);
        bool blue_present = detect_signature (vision_sensor, BLUE_SIG);
        // Update LCD only when detection state changes
        if (red_present != last_red) {
            pros::lcd::set_text(2, red_present ? "Red object detected!" : "No Red object"); 
            last_red = red_present;
        }
        if (blue_present != last_blue) {
            pros::lcd::set_text(3, blue_present ? "Blue object detected!": "No Blue object");
            last_blue = blue_present;
        }

        // Arcade drive: left stick Y for forward/back, right stick X for turning
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        left_mg.move(dir - turn);
        right_mg.move(dir + turn);
        pros::delay(20);
    }
}