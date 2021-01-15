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
      i_value = std::max(i_value+=current_error,500);
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

void set_target(pros::Imu imu){
  target = (int)imu.get_rotation()+target;
}

void reach_target(int delay, pros::Imu imu){
  while(std::abs(target-(int)imu.get_rotation())>5);
  pros::delay(delay);
}

void update(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, int tolerance, pros::Imu imu, pros::Controller con){
if (is_enabled){
      int speed = iter_pos(imu);
      FL.move(speed); BL.move(speed); FR.move(-speed); BR.move(-speed);
  }
  }

  int iter_pos(pros::Imu imu){
    int current_pos = (int)imu.get_rotation();
    current_error = target - (int)imu.get_rotation();
    return PID::update();
  }

};


class chassis_PID : public PID{
public:

  turn_PID turn = turn_PID(0,0,0,0,false);
  int current_pos;
  int correct = 0;

  chassis_PID(double kP, double kI, double kD, double turn_kP, double turn_kI, double turn_kD, int target, bool is_enabled)
  :PID(kP, kI, kD, target, is_enabled){
    turn = turn_PID(turn_kP, turn_kI, turn_kD, 0, true);

  }

  void reset(bool enable, int correction){
      prev_error = 0;
      correct = correction;
      turn.i_value = 0;
      i_value = 0;
      is_enabled = enable;
  }



  int update(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, int tolerance, pros::Imu imu){
    int turn_mod = turn.iter_pos(imu);
    current_pos = (BL.get_position()+FL.get_position()+BR.get_position()+FR.get_position()-correct)/4;
    current_error = target - current_pos;
    int speed = PID::update();
    BL.move(speed+turn_mod); FL.move(speed+turn_mod); BR.move(speed-turn_mod); FR.move(speed-turn_mod);
    return speed;
  }


};
