#include "main.h"
#include "lemlib/api.hpp"
using namespace pros;
using namespace lemlib;
//motor ports, +/- for direction
int p_l1=0;
int p_l2=0;
int p_r1=0;
int p_r2=0;
//imu port
int p_imu=0;
//left and right motorgroups
MotorGroup left({p_l1, p_l2}, v5::MotorGears::blue);
MotorGroup right({p_r1, p_l2}, v5::MotorGears::blue);
Imu imu(p_imu);
Drivetrain drivetrain(
	&left,
	&right,
	13,
	Omniwheel::NEW_325,
	600,
	2
);
Imu imu(p_imu);
