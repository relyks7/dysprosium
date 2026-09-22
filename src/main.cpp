#include "main.h"
#include "lemlib/api.hpp"
using namespace pros;
using namespace lemlib;

/// unset params
//motor ports, +/- for direction
int p_l1=0;
int p_l2=0;
int p_r1=0;
int p_r2=0;
//imu port
int p_imu=0;
//tracking wheel encoders, horizontal and vertical, - if reversed
int p_h_sens=0;
int p_v_sens=0;
//tracking wheels - offsets
double oset_h=0;
double oset_v=0;
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
///

/// drivetrain
//left and right motorgroups
MotorGroup left({p_l1, p_l2}, v5::MotorGears::blue);
MotorGroup right({p_r1, p_l2}, v5::MotorGears::blue);
Drivetrain drive(
	&left,
	&right,
	w_track,
	wheel_drive,
	rpm_drive,
	h_drift_drive
);
Imu imu(p_imu); //intertial measurement unit;
//tracking wheels + sensors + odom
Rotation h_sens(p_h_sens);
Rotation v_sens(p_v_sens);
TrackingWheel h_track_wheel(&h_sens, Omniwheel::NEW_2, oset_h);
TrackingWheel v_track_wheel(&v_sens, Omniwheel::NEW_2, oset_v);
OdomSensors odom_sens(
	&v_track_wheel,
	nullptr,
	&h_track_wheel,
	nullptr,
	&imu
);
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
	0 //max acceleration
);
///

/// chassis
Chassis chassis(drive, control_lat, control_ang, odom_sens);
///

/// init
void initialize(){
	lcd::initialize;
	chassis.calibrate();
	Task screen_task(
		[&](){
			while (true){
				lcd::print(0, "x: %f", chassis.getPose().x);
				lcd::print(1, "y: %f", chassis.getPose().y);
				lcd::print(2, "yheta: %f", chassis.getPose().theta);
				delay(20);
			}
		}
	);
}
///

//~relyks