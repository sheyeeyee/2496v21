#include "main.h"

void two_button_move(pros::Controller con, pros::controller_digital_e_t fwd_btn, pros::controller_digital_e_t rev_btn, pros::Motor m, int speed){
    if(con.get_digital(fwd_btn)){m.move(speed);}
    else if(con.get_digital(rev_btn)){m.move(-speed);}
    else{m.move(0);}
}

void tank_drive(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, pros::Controller con){
    BL.move(con.get_analog(ANALOG_LEFT_Y));
    FL.move(con.get_analog(ANALOG_LEFT_Y));
    BR.move(con.get_analog(ANALOG_RIGHT_Y));
    FR.move(con.get_analog(ANALOG_RIGHT_Y));
}

void arcade_drive(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, pros::Controller con){
    BL.move(con.get_analog(ANALOG_LEFT_Y)+con.get_analog(ANALOG_RIGHT_X));
    FL.move(con.get_analog(ANALOG_LEFT_Y)+con.get_analog(ANALOG_RIGHT_X));
    BR.move(con.get_analog(ANALOG_LEFT_Y)-con.get_analog(ANALOG_RIGHT_X));
    FR.move(con.get_analog(ANALOG_LEFT_Y)-con.get_analog(ANALOG_RIGHT_X));
}