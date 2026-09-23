#include "main.h"
#include "lemlib/api.hpp"
using namespace pros;
using namespace lemlib;

/// unset params
//motor ports, +/- for direction
int p_l1=2;
int p_l2=1;
int p_r1=4;
int p_r2=3;
///

/// set params
//track width (in)
int w_track=13;
//drivetrain wheel
float wheel_drive=Omniwheel::NEW_325;
//drivetrain rpm
double rpm_drive=600;
//drivetrain horizontal drift
double h_drift_drive=2;
//lookahead (pure pursuit)
double lookahead=15;
//timeout (pure pursuit)
double timeout=2000;
//auton file
ASSET(auton0_txt);
///

/// drivetrain
//left and right motorgroups
MotorGroup left({p_l1, p_l2}, v5::MotorGears::blue);
MotorGroup right({p_r1, p_r2}, v5::MotorGears::blue);
Drivetrain drive(
	&left,
	&right,
	w_track,
	wheel_drive,
	rpm_drive,
	h_drift_drive
);
ControllerSettings control_lat(
	10, //k_p
	0, //k_i
	3, //k_d
	3, //anti windup
	1, //small error range, dist
	100, //small error range, time
	3, //large error range, dist 
	500, //large error range, time
	20 //max acceleration
);
//angular controller
ControllerSettings control_ang(
	2, //k_p
	0, //k_i
	10, //k_d
	3, //anti windup
	1, //small error range, dist
	100, //small error range, time
	3, //large error range, dist 
	500, //large error range, time
	0 
);
OdomSensors odom_sens(
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	nullptr
);
/// chassis
Chassis chassis(drive, control_lat, control_ang, odom_sens);
///

/// init
void initialize(){
	lcd::initialize();
	chassis.calibrate();
	delay(2000);
	Task screen_task(
		[&](){
			while (true){
				lcd::print(0, "x: %f", chassis.getPose().x);
				lcd::print(1, "y: %f", chassis.getPose().y);
				lcd::print(2, "yheta: %f", chassis.getPose().theta);
				delay(25);
			}
		}
	);
}
///

/// driver control
Controller ctrler(E_CONTROLLER_MASTER);
void opcontrol(){
	while (true){
		int y_l=ctrler.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int x_r=ctrler.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(y_l, x_r);
		delay(25);
	}
}
///

/// auton (path.jerryio.com)
void autonomous(){
	chassis.setPose(0,0,0);
	chassis.follow(auton0_txt, lookahead, timeout);
}
///

//~relyks