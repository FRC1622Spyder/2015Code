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
	Accelerometer *accel;
	double maxAccel;
	double accelDiffX;
	double accelDiffY;
	float driveX;
	float curveX;
	float driveY;
	float curveY;
	float twist;
	float curveT;

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
		Spyder::ConfigVar<double> mAccel("maxAccelerationVal", 2);
		maxAccel = mAccel.GetVal();
		accel = new BuiltInAccelerometer();
		m_robotDrive = new RobotDrive(2,3,1,0);
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
		driveX = 0.0f;
		curveX = 0.0f;
		driveY = 0.0f;
		curveY = 0.0f;
		twist = 0.0f;
		curveT = 0.0f;
		accelDiffX = 0.0f;
		accelDiffY = 0.0f;
		double xAccel = accel->GetX();
		double yAccel = accel->GetY();
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
			twist = driveStick->GetTwist();
			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;

			accelDiffX = xAccel-(3*driveX*driveX);
			accelDiffY = yAccel-(3*driveY*driveY);

			/*if(accelDiffX >= maxAccel)//Acceleration throttle on the x-axis
			{
				driveX = driveX + maxAccel;//if lower, increase speed by max
				curveX = driveX;
			}
			else if(accelDiffX <= -maxAccel )
			{
				driveX = driveX - maxAccel;//if higher, decrease speed by max
				curveX = driveX;
			}
			else
			{
				curveX = driveX * driveX * driveX;//normal curve drive
			}

			if(accelDiffY >= maxAccel)//Acceleration throttle on the y-axis
			{
				driveY = driveY + maxAccel;//if lower, increase speed by max
				curveY = driveY;
			}
			else if(accelDiffY <= -maxAccel)
			{
				driveY = driveY - maxAccel;//if higher, decrease speed by max
				curveY = driveY;
			}
			else
			{
				curveY = driveY * driveY * driveY;//normal curve drive
			}*/
			curveX = driveX * driveX * driveX;
			curveY = driveY * driveY * driveY;
			curveT = twist * twist * twist;

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

