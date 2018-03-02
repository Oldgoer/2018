

#include "ArmRotate.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"


ArmRotate::ArmRotate() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	x = 0;
}

// Called just before this Command runs the first time
void ArmRotate::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ArmRotate::Execute(double direction) {
	armRotate->Set(direction);
}

// Make this return true when this Command no longer needs to run execute()
bool ArmRotate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ArmRotate::End() {
	armRotate->Set(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ArmRotate::Interrupted() {

}