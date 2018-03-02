#ifndef Elevator_H
#define Elevator_H


#include <Commands/Command.h>
#include "CustomDrive.h"
#include "WPILib.h"

class Elevator : public frc::Command {
	WPI_TalonSRX *elevator = new WPI_TalonSRX(30);

public:
	Elevator();
	void Initialize();
	void Execute(double direction);
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Elevator_H