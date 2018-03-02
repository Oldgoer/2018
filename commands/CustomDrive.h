#ifndef CustomDrive_H
#define CustomDrive_H

#include <Commands/Command.h>
#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "ADIS16448_IMU.h"

class CustomDrive : public frc::Command
{
public:
		WPI_TalonSRX *lf = new WPI_TalonSRX(4);
		WPI_TalonSRX *lr = new WPI_TalonSRX(3);
		WPI_TalonSRX *rf = new WPI_TalonSRX(1);
		WPI_TalonSRX *rr = new WPI_TalonSRX(5);
		MecanumDrive *myRobot = new MecanumDrive(*lf,*lr,*rf,*rr);

	double encoderX,encoderY;


	CustomDrive();
	//CustomDrive(double x, double y, double z, double gyroZ);
	void Initialize();
	//void Execute();
	void Execute(double x, double y, double z,double gyroZ);
	void Execute(double x, double y, double z,double gyroZ, bool controller);
	bool IsFinished();
	void End();
	void Interrupted();


};

#endif  // CustomDrive_H