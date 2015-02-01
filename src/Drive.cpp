#include "subsystem.h"
//#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

/*
 * const static int frontLeftChannel	= 2;
    const static int rearLeftChannel	= 3;
    const static int frontRightChannel	= 1;
    const static int rearRightChannel	= 0;
 *
 */

float driveX;
float driveY;

class Drive : public Spyder::Subsystem
{
private:
	RobotDrive *m_robotDrive;
	Joystick *m_driveStick;

public:
	Drive() : Spyder::Subsystem("Drive")
	{
		//Don't put WPI related init here! this gets called before HALInit()...
	}

	virtual ~Drive()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		m_robotDrive = new RobotDrive(2,3,1,0);
		m_driveStick = new Joystick(0);
		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		m_driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);
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
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			float driveX = m_driveStick->GetX() / 2;
			float driveY = m_driveStick->GetY() / 2;
			m_robotDrive->MecanumDrive_Cartesian(driveX,
					driveY, m_driveStick->GetTwist());
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

