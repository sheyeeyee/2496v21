#include "main.h"

class PID{
public:
  int target;
  int current_error;
  int i_value;
  bool is_enabled;
  double kP, kI, kD;
  int prev_error = 0;

  PID(double kP, double kI, double kD, int target, bool is_enabled){
    this -> kP = kP; this -> kI = kI; this -> kD = kD; this -> target = target; this -> is_enabled = is_enabled;
  }
public:

  void reset(bool enable){
      prev_error = 0;
      i_value =0;
      is_enabled = enable;
  }

  public:int update(){
    if(is_enabled){
      i_value+=current_error;
      int d_value = current_error - prev_error;
      prev_error = current_error;
      return current_error*kP+i_value*kI+d_value*kD;
    }
    return -1;
  }

  public:void change_target(int tar){
    target = tar;
  }

};



class pot_PID : public PID{
public:


  pot_PID(double kP, double kI, double kD, int target, bool is_enabled)
    :PID(kP, kI, kD, target, is_enabled){
  }

  void update(pros::ADIAnalogIn pot, pros::Motor m){
    if(is_enabled){
      this -> current_error = target - pot.get_value();
      m.move (PID::update());
    }
  }
};



class turn_PID : public PID{
public:

  turn_PID(double kP, double kI, double kD, int target, bool is_enabled)
  :PID(kP, kI, kD, target, is_enabled){
  }

void update(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, int tolerance, pros::Imu imu, pros::Controller con){
if (is_enabled){
      int speed = iter_pos(tolerance, imu);
      FL.move(speed); BL.move(speed); FR.move(-speed); BR.move(-speed);
  }
  }

  int iter_pos(int tolerance, pros::Imu imu){
    int current_pos = correct_imu_pos(target, tolerance, imu);
    current_error = target - current_pos;
    int speed = -PID::update();
    return PID::update();
  }

  int correct_imu_pos(int target, int tolerance, pros::Imu imu){
    if((target>0 && imu.get_heading() > 360-tolerance) || (target<0 && imu.get_heading()>0 && imu.get_heading()<tolerance))
      return 0;
    return imu.get_heading();
  }

};

class chassis_PID : public PID{
public:

  turn_PID turn = turn_PID(0,0,0,0,false);
  int current_pos;

  chassis_PID(double kP, double kI, double kD, double turn_kP, double turn_kI, double turn_kD, int target, bool is_enabled)
  :PID(kP, kI, kD, target, is_enabled){
    turn = turn_PID(turn_kP, turn_kI, turn_kD, 0, true);

  }

  void reset(bool enable){
      prev_error = 0;
      turn.i_value = 0;
      i_value =0;
      is_enabled = enable;
  }

  int update(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, int tolerance, pros::Imu imu){
    int turn_mod = turn.iter_pos(tolerance, imu);
    current_pos = (BL.get_position()+FL.get_position()+BR.get_position()+FR.get_position())/4;
    current_error = target - current_pos;
    int speed = PID::update();
    BL.move(speed+turn_mod); FL.move(speed+turn_mod); BR.move(speed-turn_mod); FR.move(speed-turn_mod);
    return speed;
  }


};
