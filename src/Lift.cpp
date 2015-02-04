#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Lift : public Spyder::Subsystem
{
private:
	    bool lift1PosButton;
		bool lift2PosButton;
		bool lift3PosButton;
public:
	Lift() : Spyder::Subsystem("Lift")
	{

	}

	virtual ~Lift()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			Spyder::TwoIntConfig firstLiftPos("LiftPos1ButtonVal", 2, 1);
			Spyder::TwoIntConfig secondLiftPos("LiftPos2ButtonVal", 2, 4);
			Spyder::TwoIntConfig thirdLiftPos("LiftPos3ButtonVal", 2, 3);
			lift1PosButton = Spyder::GetJoystick(firstLiftPos.GetVar(1))->GetRawButton(firstLiftPos.GetVar(2));
			lift2PosButton = Spyder::GetJoystick(secondLiftPos.GetVar(1))->GetRawButton(secondLiftPos.GetVar(2));
			lift3PosButton = Spyder::GetJoystick(thirdLiftPos.GetVar(1))->GetRawButton(thirdLiftPos.GetVar(2));
			Spyder::ConfigVar<uint32_t> liftMotor ("liftMotorButton", 2);
			break;
		default:
			break;
		}
	}
	virtual void Periodic(Spyder::RunModes runmode)
	{
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			if(lift1PosButton)
			{

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
Lift lift;

