#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

//float driveX;
//float driveY;

class Drive : public Spyder::Subsystem
{
private:
	RobotDrive *m_robotDrive;
	Joystick *driveStick;
	//Spyder::ConfigVar<bool> test
	//float curvature;

public:
	Drive() : Spyder::Subsystem("Drive")//, test("test", true)...
	{
		//Don't put WPI related init here! this gets called before HALInit()...
	}

	virtual ~Drive()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{

		m_robotDrive = new RobotDrive(2,3,1,0);
		//m_driveStick = new Joystick(0);
		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		m_robotDrive->SetSafetyEnabled(false);
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
		float driveX = 0.0f;
		float curveX = 0.0f;
		float driveY = 0.0f;
		float curveY = 0.0f;
		float twist = 0.0f;
		float curveT = 0.0f;
		Spyder::TwoIntConfig rightJoystick("rightJoyBind", 0, 1);
		driveStick = Spyder::GetJoystick(rightJoystick.GetVar(1));
		driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			driveX = driveStick->GetRawAxis(rightJoystick.GetVar(1));
			driveY = driveStick->GetRawAxis(rightJoystick.GetVar(2));
			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			curveX = driveX * driveX * driveX;
			curveY = driveY * driveY * driveY;
			//twist = driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);
			twist = driveStick->GetTwist();
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;
			curveT = twist * twist * twist;
			//float driveX = m_driveStick->GetX() / 2;
			//float driveY = m_driveStick->GetY() / 2;
			m_robotDrive->MecanumDrive_Cartesian(curveX, curveY, curveT);
			break;
		default:
			break;
		}
	}

	virtual void RobotInit()
	{
	}
};

Drive drive;

