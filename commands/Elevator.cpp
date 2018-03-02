#include "Elevator.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
Elevator::Elevator() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Elevator::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Elevator::Execute(double direction) {
	elevator->Set(direction);
}

// Make this return true when this Command no longer needs to run execute()
bool Elevator::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void Elevator::End() {
	elevator->Set(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Elevator::Interrupted() {

}