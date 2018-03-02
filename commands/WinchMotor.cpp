
#include "WinchMotor.h"

#include "WPILib.h"
#include "ctre/Phoenix.h"
WinchMotor::WinchMotor() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void WinchMotor::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void WinchMotor::Execute() {
	winchMotor->Set(1.0);

}

// Make this return true when this Command no longer needs to run execute()
bool WinchMotor::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void WinchMotor::End() {
	winchMotor->Set(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void WinchMotor::Interrupted() {

}