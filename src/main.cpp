#include "main.h"
#include "PID.h"
#include <cmath>
//CONSTRUCTORS

//chassis
pros::Motor front_left (1, pros::E_MOTOR_GEARSET_18);
pros::Motor front_right (10,pros::E_MOTOR_GEARSET_18,true);
pros::Motor back_left (12,pros::E_MOTOR_GEARSET_18);
pros::Motor back_right (13,pros::E_MOTOR_GEARSET_18,true);
pros::Imu imu (19);

//lift
pros::Motor lift (16,pros::E_MOTOR_GEARSET_18);
pros::ADIAnalogIn lift_pot ('B');

//intake
pros::Motor intake_left (11,pros::E_MOTOR_GEARSET_06);
pros::Motor intake_right (20,pros::E_MOTOR_GEARSET_06,true);

//roller
pros::Motor roller (17,pros::E_MOTOR_GEARSET_18,true);

//controller
pros::Controller con (CONTROLLER_MASTER);


//chassis
turn_PID turn(.65,.001,0,270,true); //.85,.003875,0,45; .79,.0025,0,90; .7,.0011245,0,180; 270; 360;

//chassis_PID go(0.14074,0.0000005,0,0,0,0,4000, true);

//lift PID
int TOP_LIFT = 1900;
int BOTTOM_LIFT = 560;
bool pid_active = true;
int current_error = 0;
int count = 0;

pot_PID lift_pid(0.5,0.000085,0,BOTTOM_LIFT,true);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();


	}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

double joystickCurve(double input, double prev, double tol){
		double yes = pow(input/20,3);
		if(prev+tol<yes){return prev+tol;}
		else if(prev-tol>yes){return prev-tol;}
		return yes;
}


void opcontrol() {
/*
	imu.reset();
	pros::delay(2300);
	while (imu.is_calibrating());

	while (true) {

		con.print(2,0,"%d",(int)imu.get_heading());

		pros::lcd::set_text(1, std::to_string(imu.get_heading()));
		turn.update(back_left, front_left, back_right, front_right, 10, imu);
*/
		double prev_power=0;
		double prev_turn =0;

		while(true){

		//	int temp = go.update(back_left, front_left, back_right, front_right,0,imu);

		//	con.print(2, 0, "value %d",go.current_pos);

		//chassis

			double power = joystickCurve(con.get_analog(ANALOG_LEFT_Y), prev_power, 15);
	    double turn = joystickCurve(con.get_analog(ANALOG_RIGHT_X), prev_turn, 15);
			prev_power = power; prev_turn = turn;
	    int left = power + turn;
	    int right = power - turn;
	    front_left.move(left);
			back_left.move(left);
	    front_right.move(right);
			back_right.move(right);

			//turn

					//error and update function is desired angle - sensored angle

//lift
			if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1))	{
				lift.move(127);
				}
			else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L2))	{
				lift.move(-127);
			}
				else {
					lift.move(0);
				}

			//lift PID
				lift_pid.update(lift_pot, lift);

				if(con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
					if(lift_pid.target == TOP_LIFT){
						lift_pid.target = BOTTOM_LIFT;
					}
					else{lift_pid.target = TOP_LIFT;}
				}

					//lift position display
						if(count%25==0){con.clear_line(2);}
						//con.print(2, 0, "value %d",lift_pot.get_value()); //modulo (%) produces remainder of integer division; %d is for decimal %i is for integer
						count++;
 			 			pros::delay(10);

//intake
			if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1))	{
				intake_left.move(127);
				intake_right.move(127);
				roller.move(127);
			}

//outtake
			else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				intake_left.move(-127);
				intake_right.move(-127);
				roller.move(-127);
			}
			else {
				intake_left.move(0);
				intake_right.move(0);
				roller.move(0);
			}

			if (con.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				intake_left.move(-127);
				intake_right.move(-127);
			}

//scoring while intaking
			 if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R2))	{
				intake_left.move(127);
 				intake_right.move(127);
				roller.move(-127);
				}

//just scoring
			if(con.get_digital(pros::E_CONTROLLER_DIGITAL_X))	{
				roller.move(-127);
			 }

		 }
	 }
