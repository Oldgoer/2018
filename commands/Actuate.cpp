#include "Actuate.h"
#include "WPILib.h"
#include "WinchMotor.h"
#include "ctre/Phoenix.h"

Actuate::Actuate(int x, int y) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	 port1 = x;
	 port2 = y;
	solenoid = new DoubleSolenoid(port1,port2);
}

// Called just before this Command runs the first time
void Actuate::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Actuate::Execute(DoubleSolenoid::Value direction) {

	solenoid->Set(direction);



}

// Make this return true when this Command no longer needs to run execute()
bool Actuate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void Actuate::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Actuate::Interrupted() {

}