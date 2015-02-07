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
	Accelerometer *accel;
	float driveX;
	float curveX;
	float driveY;
	float curveY;
	float twist;
	float curveT;
	float rampVal;

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
		Spyder::ConfigVar<float> accelRamp("driveAccelRampVal", 0.02);//GetAccelerationRamp
		rampVal = accelRamp.GetVal();

		m_robotDrive = new RobotDrive(2,3,1,0);//Configure mecanum drive
		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		driveX = 0.0f;
		curveX = 0.0f;
		driveY = 0.0f;
		curveY = 0.0f;
		twist = 0.0f;
		curveT = 0.0f;

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
		Spyder::TwoIntConfig rightJoystick("rightJoyBind", 0, 1);
		driveStick = Spyder::GetJoystick(rightJoystick.GetVar(1));
		driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);

		driveX = driveStick->GetRawAxis(rightJoystick.GetVar(1));
		driveY = driveStick->GetRawAxis(rightJoystick.GetVar(2));
		twist = driveStick->GetTwist();


		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;

		case Spyder::M_DISABLED:
			break;

		case Spyder::M_TELEOP:
		{
			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;

			if(curveY < driveY)
			{
				curveY += rampVal;
			}
			else if(curveY > driveY)
			{
				curveY -= rampVal;
			}

			if(curveX < driveX)
			{
				curveX += rampVal;
			}
			else if(curveX > driveX)
			{
				curveX -= rampVal;
			}

			if(curveT < twist)
			{
				curveT += rampVal;
			}
			else if(curveT > twist)
			{
				curveT -= rampVal;
			}

			m_robotDrive->MecanumDrive_Cartesian(curveX, curveY, curveT);
			break;
		}
		default:
			break;
		}
	}

	virtual void RobotInit()
	{
	}
};

Drive drive;

