#ifndef ArmRotate_H
#define ArmRotate_H

#include <Commands/Command.h>
#include "CustomDrive.h"
#include "WPILib.h"


class ArmRotate : public frc::Command {

	WPI_TalonSRX *armRotate = new WPI_TalonSRX(40);

public:
	double x;
	ArmRotate();
	void Initialize();
	void Execute(double direction);
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ArmRotate_H