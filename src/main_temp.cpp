#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "main.h"
#include "lemlib/api.hpp"
using namespace pros;
using namespace lemlib;
struct tickstep{
	float ly;
	float rx;
	int r1;
	int r2;
	int l1;
	int l2;
};
std::vector<tickstep> temppath{};
bool RECORDTOGGLE=false;;
bool RECORD=false;
bool a_press=false;
bool b_press=false;
/// unset params
//motor ports, +/- for direction
int p_l1=-10;
int p_l2=-1;
int p_r1=20;
int p_r2=11;
//lift ports, +/- for direction
int p_lft1=9;
int p_lft2=-19;
///
//imu potr
int p_imu=8;
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
//lift speeds, velocity
int lftspd_up=70;
int lftspd_down=30;
int drop_speed=150;
//pneumatic claw
adi::Pneumatics claw('a', false);
bool is_extend=false;
bool is_press=false;
//auton file
ASSET(autonplaybackpath_txt);
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

/// lift
//two lift motors for dr4b (might need pid, don't know yet)
Motor lft1(p_lft1, v5::MotorGears::red);
Motor lft2(p_lft2, v5::MotorGears::red);
///

/// pid
//lateral controller
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
Imu imu(p_imu); //intertial measurement unit;
OdomSensors odom_sens(
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu
);
/// chassis
Chassis chassis(drive, control_lat, control_ang, odom_sens);
///

/// init
void initialize(){
	lft1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	lft2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	lcd::initialize();
	chassis.calibrate();
	delay(2000);
	Task screen_task(
		[&](){
			while (true){
				lcd::print(0, "x: %f", chassis.getPose().x);
				lcd::print(1, "y: %f", chassis.getPose().y);
				lcd::print(2, "theta: %f", chassis.getPose().theta);
				lcd::print(3, "RECORDING HARD SWITCH IS TOGGLED?: %s", RECORDTOGGLE);
				lcd::print(3, "RECORDING IS ON?: %s", RECORD && RECORDTOGGLE);
				delay(25);
			}
		}
	);
}
///

/// control
Controller ctrler(E_CONTROLLER_MASTER);
void opcontrol(){
	while (true){
		//drive
		int y_l=ctrler.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int x_r=ctrler.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(y_l, x_r);
		//lift
		if (ctrler.get_digital(E_CONTROLLER_DIGITAL_R1)){
			lft1.move(127);
			lft2.move(127);
		} else if (ctrler.get_digital(E_CONTROLLER_DIGITAL_L1)){
			lft1.move_velocity(-lftspd_down);
			lft2.move_velocity(-lftspd_down);
		} else if (ctrler.get_digital(E_CONTROLLER_DIGITAL_R2)){
			lft1.move_velocity(-drop_speed);
			lft2.move_velocity(-drop_speed);
		} else{
			lft1.brake();
			lft2.brake();
		}
		//claw
		if (ctrler.get_digital(E_CONTROLLER_DIGITAL_L2)){
			if (!is_press){
				if (is_extend) claw.retract();
				else claw.extend();
				is_extend=!is_extend;
			}
			is_press=true;
		} else is_press=false;
		if (ctrler.get_digital(E_CONTROLLER_DIGITAL_B)){
			if (!b_press){
				RECORDTOGGLE=!RECORDTOGGLE;
			}
			b_press=true;
		} else b_press=false;
		if (RECORDTOGGLE){
			//record
			if (ctrler.get_digital(E_CONTROLLER_DIGITAL_A)){
				if (!a_press){
					RECORD=!RECORD;
					if (!RECORD){
						std::cout<<"DUMPSTART\n";
						for (auto [a, b, c, d, e, f]:temppath){
							std::cout<<a<<' '<<b<<' '<<c<<' '<<d<<' '<<e<<' '<<f<<'\n';
						}
						std::cout<<"DUMPEND\n";
						temppath.clear();
					}
				}
				a_press=true;
			} else a_press=false;
			if (RECORD){
				temppath.push_back(tickstep{y_l, x_r, ctrler.get_digital(E_CONTROLLER_DIGITAL_R1), ctrler.get_digital(E_CONTROLLER_DIGITAL_R2), ctrler.get_digital(E_CONTROLLER_DIGITAL_L1), ctrler.get_digital(E_CONTROLLER_DIGITAL_L2)});
			}
		}
		delay(25);
	}
}
///

/// auton (path.jerryio.com)
/*
idea: rerun a prerecorded path
needs:
arcade at every timestep
digital at every timestep
=> LY, RX, R1, R2, L1, L2
*/
void autonomous(){
	is_extend=false;
	is_press=false;
	std::string curline;
	std::string file_content(reinterpret_cast<char*>(autonplaybackpath_txt.buf), autonplaybackpath_txt.size);
    std::stringstream pathtestfile(file_content);
	while (getline(pathtestfile, curline)){
		std::stringstream ss(curline);
		float LY, RX;
		int R1, R2, L1, L2;
		ss>>LY>>RX>>R1>>R2>>L1>>L2;
		//drive
		int y_l=LY;
		int x_r=RX;
		chassis.arcade(y_l, x_r);
		//lift
		if (R1){
			lft1.move(127);
			lft2.move(127);
		} else if (L1){
			lft1.move_velocity(-lftspd_down);
			lft2.move_velocity(-lftspd_down);
		} else if (R2){
			lft1.move_velocity(-drop_speed);
			lft2.move_velocity(-drop_speed);
		} else{
			lft1.brake();
			lft2.brake();
		}
		//claw
		if (L2){
			if (!is_press){
				if (is_extend) claw.retract();
				else claw.extend();
				is_extend=!is_extend;
			}
			is_press=true;
		} else is_press=false;
		delay(25);
	}
}
///

//~relyks