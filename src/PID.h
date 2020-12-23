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



class chassis_PID : public PID{

  chassis_PID(double kP, double kI, double kD)
  :PID(kP, kI, kD,0,false){}

};


class turn_PID : public PID{
public:
  turn_PID(double kP, double kI, double kD, int target, bool is_enabled)
  :PID(kP, kI, kD, target, is_enabled){}

  void update(pros::Motor BL, pros::Motor FL, pros::Motor BR, pros::Motor FR, pros::Imu imu){
    int current_pos;
    if(imu.get_heading()>350){current_pos = 0;}
    else{current_pos=imu.get_heading();}

    current_error = current_pos - target;
    int speed = -PID::update();
    FL.move(speed); BL.move(speed); FR.move(-speed); BR.move(-speed);
  }
};
