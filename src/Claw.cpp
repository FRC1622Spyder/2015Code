#include "subsystem.h"
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
		Spyder::TwoIntConfig openClaw ("openClawButtonVal", 2, 7);
		Spyder::TwoIntConfig closeClaw ("closeClawButtonVal", 2, 8);
		Spyder::ConfigVar<uint32_t> extendSol("clawExtendSolPort", 0);
		Spyder::ConfigVar<uint32_t> retractSol("clawRetractSolPort", 1);
		openClawButton = Spyder::GetJoystick(openClaw.GetVar(1))->GetRawButton(openClaw.GetVar(2));
		closeClawButton = Spyder::GetJoystick(closeClaw.GetVar(1))->GetRawButton(closeClaw.GetVar(2));
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_TELEOP:
			if(openClawButton)
			{
				Spyder::GetSolenoid(extendSol.GetVal())->Set(false);
				Spyder::GetSolenoid(retractSol.GetVal())->Set(true);
			}
			if(closeClawButton)
			{
				Spyder::GetSolenoid(extendSol.GetVal())->Set(true);
				Spyder::GetSolenoid(retractSol.GetVal())->Set(false);
			}
			break;
		default:
			break;
		}
	}
	virtual void RobotInit()
	{
	}




};
