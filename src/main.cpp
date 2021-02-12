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
pros::ADIAnalogIn lift_pot ('A');

//intake
pros::Motor intake_left (11,pros::E_MOTOR_GEARSET_06);
pros::Motor intake_right (20,pros::E_MOTOR_GEARSET_06,true);

//roller
pros::Motor roller (2,pros::E_MOTOR_GEARSET_18,true);

//controller
pros::Controller con (CONTROLLER_MASTER);



//chassis_PID go(0.14074,0.0000005,0,0,0,0,4000, true);

//lift PID
int TOP_LIFT = 1900;
int BOTTOM_LIFT = 608;
bool pid_active = true;
int current_error = 0;
int count = 0;

pot_PID lift_pid(0.4,0,0,BOTTOM_LIFT,false);


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
 turn_PID turn(1.49,0,0,0,false); //.85,.003875,0,45; .79,.0025,0,90; .7,.0011245,0,180; 270; 360;
 void run_turn_PID(void* param){
 	while(1==1){
		if(turn.is_enabled){
 		turn.update(back_left, front_left, back_right, front_right, 5, imu, con);
		pros::delay(10);
	}
	else{
		pros::delay(23);
	}
 	}
}

 chassis_PID chassis(0.14074,0.0000005,0,0,0,0,0, false);

 void run_chassis_PID(void* param){
 	while(1==1){
		if(chassis.is_enabled){
 		chassis.update(back_left, front_left, back_right, front_right, 0, imu);
		pros::delay(10);
	}
	else{
		pros::delay(23);
	}
 	}
 }

 void run_lift_PID(void* param){
   while(1==1){
     if(lift_pid.is_enabled){
       lift_pid.update(lift_pot, lift);
       pros::delay(10);
     }
     else{
       pros::delay(23);
     }
   }
 }

 void print_gyro(void* param){
   con.print(2, 1, "%d", (int)imu.get_rotation());
   pros::delay(100);
 }

void initialize() {
	pros::lcd::initialize();
	pros::Task yes(run_chassis_PID);
	pros::Task no(run_turn_PID);
  pros::Task maybe(run_lift_PID);
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
void stop_motors(){
	back_left.move(0);
	front_left.move(0);
	back_right.move(0);
	front_right.move(0);
}

int correction(){
	return back_right.get_position()+front_right.get_position()+back_left.get_position()+front_left.get_position();
}

void autonomous() {
  //pros::Task so(print_gyro);
  pros::lcd::initialize();

  con.set_text(1, 1, "here");

	imu.reset();
	pros::delay(2300);
  while(imu.is_calibrating());

	stop_motors();

 //1
//deploy
  intake_right.move(-127);
  intake_left.move(-127);
  lift_pid.target=TOP_LIFT;
  lift_pid.reset(true);
  pros::delay(1000);
  lift_pid.reset(false);
  lift_pid.target=BOTTOM_LIFT;
  lift_pid.reset(true);
  pros::delay(400);
  lift_pid.reset(false);

//intake first ball
	pros::delay(1900);
	chassis.target=2300;
	chassis.reset(true, correction());
	intake_right.move(127);
	intake_left.move(127);
	roller.move(127);
  pros::delay(1400);
	intake_right.move(0);
	intake_left.move(0);
	roller.move(0);

//turn right towards corner goal
	chassis.reset(false, correction());
  turn.target=81; ///////////////////
	turn.reset(true);
  turn.reach_target(100, imu);
  turn.reset(false);

//foward a little
  chassis.target=830;
  chassis.reset(true, correction());
  pros::delay(800);
  chassis.reset(false, correction());

//lift to score
  lift_pid.target=TOP_LIFT;
  lift_pid.reset(true);
  pros::delay(1000);

//scoring while descoring in corner
  chassis.target=805;
  chassis.reset(true, correction());
  intake_right.move(127);
  intake_left.move(127);
  pros::delay(1900);
  roller.move(-127);
  pros::delay(300);
  roller.move(0);
  pros::delay(500);
  intake_left.move(0);
  intake_right.move(0);
  pros::delay(1000);

//backing out and outtaking
  intake_right.move(127);
  intake_left.move(127);
  pros::delay(200);
  chassis.target=-600;
  chassis.reset(true, correction());
  intake_right.move(127);
  intake_left.move(127);
  pros::delay(1000);
  intake_right.move(0);
  intake_left.move(0);
  chassis.target=-1000;
  chassis.reset(false, correction());

//lift down
  turn.target=45; ///////////////////
  turn.reset(true);
  pros::delay(500);
  turn.reset(false);
  chassis.target=-2000;
  chassis.reset(true, correction());
  intake_right.move(-127);
  intake_left.move(-127);
  pros::delay(1800);
  lift_pid.target=BOTTOM_LIFT;
  lift_pid.reset(true);
  pros::delay(300);
  intake_right.move(0);
  intake_left.move(0);
  chassis.target=-1990;
  chassis.reset(true, correction());
  pros::delay(600);
  chassis.reset(false, correction());
  turn.target=-55; ///////////////////
  turn.reset(true);
  turn.reach_target(400, imu);
  turn.reset(false);
  pros::delay(300);

 //2
//descore center v1
  chassis.target=6400;
  chassis.reset(true, correction());
  pros::delay(1000);
  chassis.target=-1000;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.target=1100;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.target=-1000;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.target=1100;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.target=-1000;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.target=1100;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.reset(false, correction());
  turn.target=-50; ///////////////////
  turn.reset(true);
  pros::delay(400);
  turn.reset(false);
  chassis.target=-900;
  chassis.reset(true, correction());
  pros::delay(500);
  chassis.reset(false, correction());

//score center
  lift_pid.target=TOP_LIFT;
  lift_pid.reset(true);
  turn.target=-48; ///////////////////
  turn.reset(true);
  pros::delay(400);
  turn.reset(false);
  pros::delay(600);
  chassis.target=1300;
  chassis.reset(true, correction());
  pros::delay(1000);
  chassis.reset(false, correction());
  roller.move(-127);
  chassis.target=500;
  pros::delay(1600);
  roller.move(0);

//reset pos
  chassis.target=-5000;
  chassis.reset(true, correction());
  intake_left.move(-127);
  intake_right.move(-127);
  pros::delay(1500);
  intake_left.move(0);
  intake_right.move(0);
  lift_pid.target=BOTTOM_LIFT;
  pros::delay(900);
  imu.reset();
  pros::delay(2100);

 //3
//go to next corner
  chassis.target=1850;
  chassis.reset(true, correction());
  pros::delay(700);
  chassis.reset(false, correction());
  turn.target=-90; ///////////////////
  turn.reset(true);
  pros::delay(800);
  turn.reset(false);
  chassis.target=4073;
  chassis.reset(true, correction());
  pros::delay(1700);
  chassis.reset(false, correction());
  turn.target=0; ///////////////////
  turn.reset(true);
  pros::delay(1100);
  turn.reset(false);
  chassis.target=6600;
  chassis.reset(true, correction());

//intake next ball
  intake_left.move(127);
  intake_right.move(127);
  roller.move(127);
  pros::delay(3000);
  intake_left.move(0);
  intake_right.move(0);
  roller.move(0);

//turn towards corner
  chassis.target=-1800;
  chassis.reset(true, correction());
  pros::delay(900);
  chassis.reset(false, correction());
  turn.target=-44; ///////////////////
  turn.reset(true);
  pros::delay(700);
  turn.reset(false);

//score corner
  lift_pid.target=TOP_LIFT;
  lift_pid.reset(true);
  pros::delay(600);
  chassis.target=2200;
  chassis.reset(true, correction());
  pros::delay(700);
  intake_right.move(127);
  intake_left.move(127);
  pros::delay(2000);
  roller.move(-127);
  pros::delay(300);
  roller.move(0);
  pros::delay(500);
  intake_left.move(0);
  intake_right.move(0);
  pros::delay(1000);

//backing out
  chassis.target=-800;
  chassis.reset(true, correction());
  intake_right.move(127);
  intake_left.move(127);
  pros::delay(2000);
  intake_right.move(0);
  intake_left.move(0);
  pros::delay(100);
  chassis.reset(false, correction());

}
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


  chassis.reset(false, 0);
  turn.reset(false);
  lift_pid.reset(true);

	while (true) {



		pros::lcd::set_text(1, std::to_string(imu.get_rotation()));
		//turn.update(back_left, front_left, back_right, front_right, 10, imu);

		double prev_power=0;
		double prev_turn =0;

		while(true){

		//	int temp = go.update(back_left, front_left, back_right, front_right,0,imu);

		//	con.print(2, 0, "value %d",go.current_pos);

		//chassis

			double power = joystickCurve(con.get_analog(ANALOG_LEFT_Y), prev_power, 15);
	    double turn = joystickCurve(con.get_analog(ANALOG_RIGHT_X), prev_turn, 15);
			prev_power = power; prev_turn = turn;
	    int left = power + 0.3*turn;
	    int right = power - 0.3*turn;
	    front_left.move(left);
			back_left.move(left);
	    front_right.move(right);
			back_right.move(right);

			//turn

					//error and update function is desired angle - sensored angle

			//lift PID
			//	lift_pid.update(lift_pot, lift);

				if(con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
					if(lift_pid.target == TOP_LIFT){
						lift_pid.target = BOTTOM_LIFT;
					}
					else{lift_pid.target = TOP_LIFT;}
				}

					//lift position display
						if(count%25==0){con.clear_line(2);}
						con.print(2, 0, "value %d",lift_pot.get_value()); //modulo (%) produces remainder of integer division; %d is for decimal %i is for integer
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
 }
