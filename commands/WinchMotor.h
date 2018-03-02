#ifndef WinchMotor_H
#define WinchMotor_H

#include <Commands/Command.h>
#include "CustomDrive.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
class WinchMotor : public frc::Command  {


public:
	//WPI_TalonSRX *referenceWinch;
	WPI_TalonSRX *winchMotor = new WPI_TalonSRX(20);
	WinchMotor();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // WinchMotor_H