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
	float lastOutputY;

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
		Spyder::ConfigVar<double> mAccel("maxAccelerationVal", 0.5);//create max acceleration value if we are using one
		Spyder::ConfigVar<uint32_t> leftFrontMotor("leftFrontDriveMotor", 2);
		//leftTopMotor = Spyder::GetVictor(leftFrontMotor.GetVal());
		//accel = new BuiltInAccelerometer();
		m_robotDrive = new RobotDrive(2,3,1,0);//initialize mecanum drive with ports 2,3,1,0. MAKE INTO CONFIG VARS
		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		driveX = 0.0f;
		curveX = 0.0f;
		//lastOutputY = 0.0f;
		driveX = 0.0f;//Store input of joystick to set speed of motors
		curveX = 0.0f;//speed setting after curve
		driveY = 0.0f;
		curveY = 0.0f;
		twist = 0.0f;//twist of joystick
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
		driveStick = Spyder::GetJoystick(rightJoystick.GetVar(1));//setting joystick values
		lastOutputY = 0.0f;

		//float rampY = 0.0f;
		//double xAccel = accel->GetX();
		//double yAccel = accel->GetY();
		driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);//setting twist axis

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
			//rampY = leftTopMotor->Get();
			driveX = driveStick->GetRawAxis(rightJoystick.GetVar(1));
			driveY = driveStick->GetRawAxis(rightJoystick.GetVar(2));//Setting axis of joystick
			twist = driveStick->GetTwist();

			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;

			//if(curveY < driveY)
			if(driveX > 0)
			{
				if(curveX < driveX)
				{
					curveX += 0.01;
				}
			}

			if(driveX < 0)
			{
				if(curveX > driveX)
				{
					curveX -= 0.01;
				}
			}

			//Attempted controlled acceleration thingy. DOES NOT WORK. REDO/FIX
			/*if(curveY < driveY)
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
				curveY = driveY;
			}/*

			if(curveT < twist)
			{
				curveT += rampVal;
			}
			else if(curveT > twist)
			{
				curveT -= rampVal;
			}

			m_robotDrive->MecanumDrive_Cartesian(curveX, curveY, curveT);
			}*/
			//curveY = driveY * driveY * driveY;

			curveX = driveX * driveX * driveX;//curve for axis
			curveT = twist * twist * twist;

			m_robotDrive->MecanumDrive_Cartesian(curveX, curveY, curveT);//setting mecanum drive with curved values

			lastOutputY = curveY;//Record last output of Y
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

