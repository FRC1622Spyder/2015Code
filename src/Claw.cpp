/*#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Claw : public Spyder::Subsystem
{
private:
	bool openClawButton;
	bool closeClawButton;
	DoubleSolenoid *clawSol;

public:
	Claw() : Spyder::Subsystem("Claw")
	{
		//DO NOT INITIALIZE WPI/CONFIG RELATED VALUES HERE
	}
	virtual ~Claw()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		Spyder::TwoIntConfig clawSolPorts("clawExtendSolPorts", 0 , 1);//Initialize the solenoids!
		//clawSol = new DoubleSolenoid (clawSolPorts.GetVar(1), clawSolPorts.GetVar(2));
		clawSol = new DoubleSolenoid (7, 0, 1);

		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_TELEOP:
			break;
		default:
			break;
		}
	}
	virtual void Periodic(Spyder::RunModes runmode)
	{
		Spyder::TwoIntConfig openClaw ("openClawButtonVal", 2, 7);//Initialize the claw ports
		Spyder::TwoIntConfig closeClaw ("closeClawButtonVal", 2, 8);

		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_TELEOP:
			openClawButton = Spyder::GetJoystick(openClaw.GetVar(1))->GetRawButton(openClaw.GetVar(2));//Get Button Vals
			closeClawButton = Spyder::GetJoystick(closeClaw.GetVar(1))->GetRawButton(closeClaw.GetVar(2));
			if(openClawButton)//Open Claw
			{
				clawSol->Set(DoubleSolenoid::kForward);
			}

			if(closeClawButton)//Close Claw
			{
				clawSol->Set(DoubleSolenoid::kReverse);
			}
			break;
		default:
			break;
		}
	}
	virtual void RobotInit()
	{
	}




};*/
