#include "CustomDrive.h"
#include "DriveController.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
CustomDrive::CustomDrive() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	//gyro = new ADIS16448_IMU;
	//individually set safety off, necessay or motors stutters
	encoderX = 0.0;
	encoderY = 0.0;
	myRobot->DriveCartesian(0.0,0.0,0.0,0.0);
}

// Called just before this Command runs the first time
void CustomDrive::Initialize() {

	//left motor configured for x direction
	lf->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	lf->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
	lf->SetSelectedSensorPosition(0,0,0);
	//right motor configured for y direction
	rf->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	rf->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
	rf->SetSelectedSensorPosition(0,0,0);
	//myRobot->SetExpiration(50.0);
	myRobot->SetSafetyEnabled(false);
	lf->SetSafetyEnabled(false);
	rf->SetSafetyEnabled(false);
	rr->SetSafetyEnabled(false);
	lr->SetSafetyEnabled(false);

	myRobot->DriveCartesian(0.0,0.0,0.0,0.0);


}


//overloaded execite for use in autonomous
void CustomDrive::Execute(double x, double y, double z, double gyroZ) {
	encoderX = lf->GetSelectedSensorPosition(0);
	encoderY = rf->GetSelectedSensorPosition(0);
		myRobot->DriveCartesian(x,y,z, gyroZ);
}
void CustomDrive::Execute(double x, double y, double z, double gyroZ, bool controller){
	encoderX = lf->GetSelectedSensorPosition(0);
	encoderY = rf->GetSelectedSensorPosition(0);
	myRobot->DriveCartesian(x,y,z, gyroZ);

}


// Make this return true when this Command no longer needs to run execute()
bool CustomDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void CustomDrive::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CustomDrive::Interrupted() {

}