#include "main.h"
#include "lemlib/api.hpp"
pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);

pros::Motor firstStageIntake(-17, pros::MotorGearset::blue);
pros::Motor basketRoller(2, pros::MotorGearset::blue);
pros::Motor hood(10, pros::MotorGearset::blue);
pros::Optical optical(1);//adjust port
pros::Imu imu(14);





// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.2, // track width
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2 // horizontal drift
);

lemlib::ControllerSettings linearController(10, // kP
                                            0, // kI
                                            3, // kD
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // kP
                                             0, // kI
                                             10, // kD
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr,
                            nullptr, // horizontal tracking wheel
                            nullptr, 
                            &imu // inertial sensor
);


lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

pros::ADIDigitalOut basket('A');
bool basketExtended = false;

pros::ADIDigitalOut matchload('B');
bool matchloadOn = false;

enum class Mode {
    Idle,
    IntakeToBasket,
    ScoreTop,
    ScoreMid,
    ScoreLow,
    Unjam,
    BottomLoad,
    ejectBall
};


enum class BallColor { Red, Blue, Unknown};

BallColor targetColor = BallColor::Red; 
struct HueRange { //NOTE do i even need htis 
    double min;
    double max;
    bool contains(double hue) const {
        if (min <= max) {
            return hue >= min && hue <= max;
        } else {
            return hue >= min || hue <= max;
        }
    }
};
//NOTE do i need second range 
const HueRange RED_RANGE1 {345.0, 360.0};//tune this 
const HueRange RED_RANGE2 {0.0, 15.0};
const HueRange BLUE_RANGE {200.0, 260.0};
const HueRange BLUE_RANGE2 {100.0, 160.0};

BallColor ballColor = BallColor::Unknown;//init
BallColor identifyColor() { 
    double hue = optical.get_hue();
    if (RED_RANGE1.contains(hue) || RED_RANGE2.contains(hue)) {
        return BallColor::Red;
    }
    if ((BLUE_RANGE.contains(hue)) || BLUE_RANGE2.contains(hue)){
        return BallColor::Blue;
    }
    return BallColor::Unknown;
}
/* 
BallColor identifyColor() {
    pros::c::optical_rgb_s_t color = optical.get_rgb();
    if (color.red > color.blue && color.red > color.green) return BallColor::Red;
    if (color.blue > color.red && color.blue > color.green) return BallColor::Blue;
    return BallColor::Unknown;
}
*/





Mode currentMode = Mode::Idle;

void handleL1Press() { // Intake to basket w color sort 
    //currentMode = (currentMode == Mode::IntakeToBasket) ? Mode::Idle : Mode::IntakeToBasket;
    if (ballColor != targetColor) {
        currentMode = (currentMode == Mode::ejectBall) ? Mode::Idle : Mode::ejectBall;
    }
    else {
        currentMode = (currentMode == Mode::IntakeToBasket) ? Mode::Idle : Mode::IntakeToBasket;
    }
}


void handleR1Press() { // Score top goal
    currentMode = (currentMode == Mode::ScoreTop) ? Mode::Idle : Mode::ScoreTop;
}

void handleR2Press() { // Score mid goal
    currentMode = (currentMode == Mode::ScoreMid) ? Mode::Idle : Mode::ScoreMid;
}

void handleBPress() { // Score low goal
    currentMode = (currentMode == Mode::ScoreLow) ? Mode::Idle : Mode::ScoreLow;
}

void handleLeftPress() { // Bottom load
    currentMode = (currentMode == Mode::BottomLoad) ? Mode::Idle : Mode::BottomLoad;
}

void handleL2Held(bool pressed) { // Unjam
    if (pressed) {
        currentMode = Mode::Unjam;
    } else if (currentMode == Mode::Unjam) {
        currentMode = Mode::Idle;
    }
}

void checkButtons() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) handleL1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) handleR1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) handleR2Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))  handleBPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) handleLeftPress();

    handleL2Held(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
}



void intakeControl() {
    while (true) {
        switch (currentMode) {
            case Mode::Idle:
                firstStageIntake.move_velocity(0);
                hood.move_velocity(0);
                basketRoller.brake();
                break;

            case Mode::IntakeToBasket:
                ;
                firstStageIntake.move_velocity(600);
                hood.move_velocity(600);
                basketRoller.brake();
                basketExtended = false;
                basket.set_value(basketExtended);
                
                break;

            case Mode::ScoreTop:
                firstStageIntake.move_velocity(600);
                basketRoller.move_velocity(600);
                hood.move_velocity(600);
                basketExtended = true;
                basket.set_value(basketExtended);
                break;

            case Mode::ScoreMid:
                basketRoller.move_velocity(600);
                firstStageIntake.move_velocity(0);
                hood.move_velocity(-600);
                basketExtended = true;
                basket.set_value(basketExtended);
                break;

            case Mode::ScoreLow:
                basketRoller.move_velocity(600);
                firstStageIntake.move_velocity(-600);
                hood.move_velocity(0);
                basketExtended = true;
                basket.set_value(basketExtended);
                break;

            case Mode::Unjam:
                basketRoller.move_velocity(-600);
                firstStageIntake.move_velocity(-600);
                hood.move_velocity(-600);
                basketExtended = false;
                basket.set_value(basketExtended);
                break;
            
            case Mode::BottomLoad:
                firstStageIntake.move_velocity(600);
                hood.move_velocity(0);
                basketRoller.move_velocity(-600);
                basketExtended = false;
                basket.set_value(basketExtended);
                break;
            case Mode::ejectBall:
                //write this idk 
                basketExtended = false;
                basket.set_value(basketExtended);
                break;
        }
        pros::delay(20);
    }
}

void toggleTask() {
    while (true) {
        checkButtons();
        pros::delay(20);
    }
}



void colorSortTask() {
    while (true) {
        ballColor = identifyColor(); 
        pros::delay(20); 
    }
}

void pneumaticControl() {
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            matchloadOn = !matchloadOn;
            matchload.set_value(matchloadOn);
        }

        pros::delay(20);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    basketRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        /*
        
        
                printf("Red value: %lf \n", rgb_value.red);
    printf("Green value: %lf \n", rgb_value.green);
    printf("Blue value: %lf \n", rgb_value.blue);
    printf("Clear value: %lf \n", rgb_value.clear);
                pros::delay(20);
        */
        
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "ball color %f", ballColor);
            pros::lcd::print(3, "hue optical %f", optical.get_hue());
			lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    pros::Task intake_task(intakeControl);
    pros::Task toggle_task(toggleTask);
    pros::Task pneumatic_task(pneumaticControl);
    pros::Task color_sort_task(colorSortTask);
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);
        pros::delay(10);
    }
}
