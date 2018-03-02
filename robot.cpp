/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>

#include "Commands/Auto_1.h"
#include "Commands/Auto_2.h"
#include "Commands/Auto_3.h"
#include "Commands/Auto_4.h"
#include "Commands/Actuate.h"
#include "Commands/CustomDrive.h"
#include "Commands/DriveController.h"
#include "Commands/WinchMotor.h"
#include "Commands/Elevator.h"
#include "Commands/ArmRotate.h"
#include "WPILIb.h"
#include "ADIS16448_IMU.h"

class Robot : public frc::TimedRobot {
	ADIS16448_IMU *gyro;
public:
	//declarations of commands, values assigned in robotInit
	CustomDrive* customDrive = NULL;
	Actuate* grabber = NULL;
	Actuate* tiltRobot = NULL;
	Actuate* liftEncoder = NULL;

	DriveController* driveController = NULL;
	Compressor *c = new Compressor();

	WinchMotor *winch = NULL;
	Elevator *elev = NULL;
	ArmRotate *tiltCube = NULL;
	//variable for solenoid direction, 0 = fwd, 1 = rverse, 2 = off
	DoubleSolenoid::Value Order66[3] = {DoubleSolenoid::kForward,DoubleSolenoid::kReverse,DoubleSolenoid::kOff };
	Joystick* driveJoy = new Joystick(0);
	Joystick* buttonJoy = new Joystick(1);
	Joystick* buttonPad = new Joystick(2);
	XboxController* driveCont = new XboxController(3);
	XboxController* buttonCont = new XboxController(4);
	//CameraServer *camera;

	double xin, yin, zin = 0.0; //initialize variables for deadzone controls
	double deadzoneX = 0.1; //edit these for deadzone controls (x) (side to side)
	double deadzoneY = 0.1; //edit these for deadzone controls	(y) (forward back)
	double deadzoneZ = 0.30; //edit these for deadzone controls (z) (twist)

	double xCont, yCont, zCont = 0.0;

	double time = 0.0; //reset pin on clime timer
	bool climbing = false;

	//************************************************************************************************************

	void RobotInit() override {



		//assigning ports to talons and solenoids, initializing drive functions
		customDrive = new CustomDrive();
		grabber = new Actuate(2,3);
		tiltRobot = new Actuate(0,1);
		liftEncoder = new Actuate(4,5);
		driveController = new DriveController();
		winch = new WinchMotor();
		elev = new Elevator();
		tiltCube = new ArmRotate();
		c->SetClosedLoopControl(true);

		//camera = new CameraServer;
		//camera->GetInstance()->StartAutomaticCapture();

		gyro = new ADIS16448_IMU;



		//adding buttons to dashboard for auto selection
		m_chooser.AddDefault("Auto Line",  &auto1);
		m_chooser.AddObject("Auto Left",  &auto2);
		m_chooser.AddObject("Auto Middle",&auto3);
		m_chooser.AddObject("Auto Right", &auto4);
		frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

		c_chooser.AddDefault("Controller", &controllerChooser);
		c_chooser.AddObject("Joystick", &joystickChooser);
		frc::SmartDashboard::PutData("Controller Selector", &c_chooser);

		customDrive->myRobot->SetSafetyEnabled(false);
		frc::SmartDashboard::PutData("IMU", gyro);
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
		frc::SmartDashboard::PutNumber("encCountY", customDrive->rf->GetSelectedSensorPosition(0));
						frc::SmartDashboard::PutData("IMU", gyro);
						//frc::SmartDashboard::PutBoolean("saftey", customDrive->myRobot->IsSafetyEnabled());
						frc::SmartDashboard::PutNumber("timeout", customDrive->myRobot->GetExpiration());
						frc::SmartDashboard::PutNumber("GyroAngle", gyro->GetAngleZ());

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {


		m_autonomousCommand = m_chooser.GetSelected();

		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Start();
		}
		customDrive->lf->SetSelectedSensorPosition(0,0,0);
		customDrive->rf->SetSelectedSensorPosition(0,0,0);
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		double gyro_Z = gyro->GetAngleZ();
		frc::SmartDashboard::PutNumber("GyroAngle", gyro_Z);
		frc::SmartDashboard::PutNumber("encCountX", customDrive->lf->GetSelectedSensorPosition(0));
				frc::SmartDashboard::PutNumber("encCountY", customDrive->rf->GetSelectedSensorPosition(0));
				frc::SmartDashboard::PutData("IMU", gyro);
				//frc::SmartDashboard::PutBoolean("saftey", customDrive->myRobot->IsSafetyEnabled());
				frc::SmartDashboard::PutNumber("timeout", customDrive->myRobot->GetExpiration());
				frc::SmartDashboard::PutNumber("GyroAngle", gyro->GetAngleZ());
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		customDrive->Initialize();
		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Cancel();
			m_autonomousCommand = nullptr;
		}
		gyro->Calibrate();
		gyro->Reset();
		time = 0.0;
		customDrive->myRobot->SetSafetyEnabled(false);
		customDrive->lf->SetSafetyEnabled(false);
		customDrive->rf->SetSafetyEnabled(false);
		customDrive->rr->SetSafetyEnabled(false);
		customDrive->lr->SetSafetyEnabled(false);
		c_controllerCommand = c_chooser.GetSelected();
		liftEncoder->Execute(Order66[1]);
	}

	void deadZone(){

		//below is custom deadzSone code.  we check that the joystick values are greater than the deadzone value.
				//if it is greater than the deadzone then it will pass the value to a variable
				//we then reinitialize the variable to a zero value so the speed isn't sudden

				if (driveJoy->GetX() > deadzoneX || driveJoy->GetX() < -deadzoneX){  //x axis deadzone check
						xin = driveJoy->GetX();

						if (xin < 0.0){
							xin = xin + deadzoneX;
							xin = xin*0.9;
						} //re initialize positive x values

						else if (xin > 0.0) {
							xin = xin - deadzoneX;
							xin = xin*0.9;
						} //re initialize negative x values
					}
					else xin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value


					if (driveJoy->GetY() > deadzoneY || driveJoy->GetY() < -deadzoneY){ //y axis deadzone check
						yin = (driveJoy->GetY());

						if (yin<0.0){
							yin = yin + deadzoneY;
							yin=yin*0.9;
						}//re initialize positive y values

						else if (yin > 0.0) {
							yin = yin - deadzoneY;
							yin=yin*0.9;
						}//re initialize negative y values
					}
					else yin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value


					if (driveJoy->GetZ()> (deadzoneZ)|| driveJoy->GetZ()< (-deadzoneZ)){ //z axis deadzone check -- note the z axis needed custom values depending on joystick
						zin = driveJoy->GetZ();
						if (zin<0.0){
							zin = zin + deadzoneZ;
							zin = zin * 0.9; //speed nerf for rotation
						}//re initialize positive z values
						else if (zin > 0.0) {
							zin = zin - deadzoneZ;
							zin = zin * 0.9; //speed nerf for rotation
						}//re initialize negative z values
					}
					else zin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value



	}

	void ControllerNerf(){

		zCont = driveCont->GetRawAxis(4);
		yCont = driveCont->GetRawAxis(1);
		xCont = driveCont->GetRawAxis(0);

		if (zCont <= 0)  {
			zCont = (zCont*zCont)*-1.0;
		}
		else zCont = (zCont*zCont)*1.0;

	}
	void TeleopPeriodic() override
	{
		frc::Scheduler::GetInstance()->Run();
		frc::SmartDashboard::PutNumber("encCountX", customDrive->lf->GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutNumber("encCountY", customDrive->rf->GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutData("IMU", gyro);
		//frc::SmartDashboard::PutBoolean("saftey", customDrive->myRobot->IsSafetyEnabled());
		frc::SmartDashboard::PutBoolean("drivesafty", customDrive->myRobot->IsSafetyEnabled());
		frc::SmartDashboard::PutBoolean("lf", customDrive->myRobot->IsSafetyEnabled());
		frc::SmartDashboard::PutBoolean("rf", customDrive->myRobot->IsSafetyEnabled());
		frc::SmartDashboard::PutBoolean("rr", customDrive->myRobot->IsSafetyEnabled());
		frc::SmartDashboard::PutBoolean("lr", customDrive->myRobot->IsSafetyEnabled());
		double gyro_Z = gyro->GetAngleZ();
		frc::SmartDashboard::PutNumber("GyroAngle", gyro_Z);
		frc::SmartDashboard::PutNumber("LF", customDrive->lf->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("RF", customDrive->rf->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("LR", customDrive->lr->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("RR", customDrive->rr->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("LFc", customDrive->lf->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("RFc", customDrive->rf->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("LRc", customDrive->lr->GetOutputCurrent());
		frc::SmartDashboard::PutNumber("RRc", customDrive->rr->GetOutputCurrent());




		climbing = false;
		liftEncoder->Execute(Order66[1]);// at the beggining the encoder will lift so it wont be damaged

		//if (c_controllerCommand == &joystickChooser || c_chooser.GetSelected() == &joystickChooser);
		c_controllerCommand = c_chooser.GetSelected();
		if (c_controllerCommand == &joystickChooser || c_chooser.GetSelected() == &joystickChooser)
		{
			deadZone();
			customDrive->Execute(xin, -yin, zin, -gyro->GetAngleZ());
		}
		else
		{
			ControllerNerf();
			customDrive->Execute(xCont,-yCont,zCont,-gyro->GetAngleZ(),true);
		}
		//reset gyro
		if (driveJoy->GetRawButton(1) || driveCont->GetRawButton(7))
		{
			gyro->Reset();
		}

		//close grabbers
		if (buttonJoy->GetRawButton(12)||buttonCont->GetRawButton(6))
		{
			grabber->Execute(Order66[0]);
		}
		//open grabbers
		if (buttonJoy->GetRawButton(11)||buttonCont->GetRawButton(5))
		{
			grabber->Execute(Order66[1]);
		}

		//winch (lift robot)
		if (buttonPad->GetRawButton(4) && buttonPad->GetRawButton(5))
		{
			winch->Execute();
		}
		else
		{
			winch->End();
		}


		//code to pull tilt pins back to robot after climb
		if(winch->winchMotor->GetOutputCurrent() >= 40)
		{
			time ++;
		}
		if(time >= 20.0)
		{
			climbing = true;
		}
		if(climbing)
		{
			tiltRobot->Execute(Order66[1]);
			time = 0.0;
		}


		//tilt the robot
		if (buttonPad->GetRawButton(3) && buttonPad->GetRawButton(6))
		{
			tiltRobot->Execute(Order66[0]);
		}
		//untilt the robot
		if (buttonPad->GetRawButton(1))
		{
			tiltRobot->Execute(Order66[1]);
		}

		//tilt the cube
		if (buttonJoy->GetRawButton(10)||buttonCont->GetRawButton(2))
		{
			tiltCube->Execute(1.0);
		}
		//untilt the cube
		else if (buttonJoy->GetRawButton(9)||buttonCont->GetRawButton(3))
		{
			tiltCube->Execute(-1.0);
		}
		else
		{
			tiltCube->End();
		}

		//elevator down
		if (buttonJoy->GetRawButton(4)||buttonCont->GetRawButton(1))//nerfed so we dont derail XV
		{
			elev->Execute(0.5);
		}
		//elevator up
		else if (buttonJoy->GetRawButton(6)||buttonCont->GetRawButton(4))
		{
			elev->Execute(-1.0);
		}
		else
		{
			elev->End();
		}


	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc::Command* m_autonomousCommand = nullptr;

	Auto_1 auto1;
	Auto_2 auto2;
	Auto_3 auto3;
	Auto_4 auto4;
	frc::SendableChooser<frc::Command*> m_chooser;

	frc::Command* c_controllerCommand = nullptr;

	DriveController joystickChooser;
	DriveController controllerChooser;
	frc::SendableChooser<frc::Command*> c_chooser;
};

START_ROBOT_CLASS(Robot)