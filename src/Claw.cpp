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
	Solenoid *clawSolExtend;
	Solenoid *clawSolRetract;

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
		Spyder::ConfigVar<uint32_t> extendSol("clawExtendSolPort", 0);//Initialize the solenoids!
		Spyder::ConfigVar<uint32_t> retractSol("clawRetractSolPort", 1);
		clawSolExtend = new Solenoid (extendSol.GetVal());
		clawSolRetract = new Solenoid (retractSol.GetVal());

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
				clawSolExtend->Set(false);//Set false solenoids before setting true
				clawSolRetract->Set(true);
			}
			if(closeClawButton)//Close Claw
			{
				clawSolRetract->Set(false);
				clawSolExtend->Set(true);
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
