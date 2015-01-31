#include "subsystem.h"
//#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Drive : public Spyder::Subsystem
{
private:
	RobotDrive *m_robotDrive;
	Joystick *m_driveStick;

public:
	Drive() : Spyder::Subsystem("Drive"),
	m_robotDrive(),
	m_driveStick()
	{
	}

	virtual ~Drive()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_TELEOP:
			/*m_robotDrive = new RobotDrive(0,1,2,3);
			m_driveStick = new Joystick(1);
			m_driveStick->SetAxisChannel(Joystick::kTwistAxis, 3);*/
			break;
		default:
			break;
		}
	}

	virtual void Periodic(Spyder::RunModes runmode)
	{
		//m_robotDrive = new RobotDrive(leftFrontVictor->GetChannel(),leftBackVictor->GetChannel(),rightFrontVictor->GetChannel(),rightBackVictor->GetChannel());
		m_robotDrive = new RobotDrive(0,1,2,3);
		m_driveStick = new Joystick(1);
		m_driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			m_robotDrive->MecanumDrive_Cartesian(m_driveStick->GetX(), m_driveStick->GetY(), m_driveStick->GetTwist());
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

