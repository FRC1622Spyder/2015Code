#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Drive : public Spyder::Subsystem
{
private:
	RobotDrive *m_robotDrive;
	Joystick *driveStick;
	CANTalon *frontLeftMotor;
	CANTalon *frontRightMotor;
	CANTalon *backLeftMotor;
	CANTalon *backRightMotor;
	//Accelerometer *accel;

	float twistSense;
	float zMultiplier;
	bool setTwistSense;

	float driveX;
	float curveX;
	float driveY;
	float curveY;
	float twist;
	float curveT;
	float curveZ;
	float rampVal;

	float xVel;
	float yVel;
	float twistVel;

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
		Spyder::ConfigVar<int> leftFrontCAN("driveLeftFrontCAN_id", 5);
		Spyder::ConfigVar<int> rightFrontCAN("driveRightFrontCAN_id", 2);
		Spyder::ConfigVar<int> leftBackCAN("driveLeftBackCAN_id", 4);
		Spyder::ConfigVar<int> rightBackCAN("driveRightBackCAN_id", 1);

		frontLeftMotor = new CANTalon(leftFrontCAN.GetVal());
		frontRightMotor = new CANTalon(rightFrontCAN.GetVal());
		backLeftMotor = new CANTalon(leftBackCAN.GetVal());
		backRightMotor = new CANTalon(rightBackCAN.GetVal());

		rampVal = accelRamp.GetVal();

		m_robotDrive = new RobotDrive(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);//Configure mecanum drive

		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		twistSense = 0.0f;
		zMultiplier = 1.0f;
		setTwistSense = false;

		driveX = 0.0f;//Store input of joystick to set speed of motors
		curveX = 0.0f;//speed setting after curve
		driveY = 0.0f;
		curveY = 0.0f;
		twist = 0.0f;
		curveT = 0.0f;

		xVel = 0.0f;
		yVel = 0.0f;
		twistVel = 0.0f;

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
		Spyder::TwoIntConfig senseButton("driveTwistSensitivitySetButton",0, 3);
		Spyder::TwoIntConfig twistSenseJoy("twistSensitivitySliderBind", 0, 4);
		driveStick = Spyder::GetJoystick(rightJoystick.GetVar(1));
		driveStick->SetAxisChannel(Joystick::kTwistAxis, 2);

		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;

		case Spyder::M_DISABLED:
			break;

		case Spyder::M_TELEOP:
		{
			driveX = driveStick->GetRawAxis(rightJoystick.GetVar(1));
			driveY = driveStick->GetRawAxis(rightJoystick.GetVar(2));//Setting axis of joystick
			twist = driveStick->GetTwist();

			setTwistSense = Spyder::GetJoystick(senseButton.GetVar(1))->GetRawButton(senseButton.GetVar(2));//Twist sensitivity settings!
			twistSense = Spyder::GetJoystick(twistSenseJoy.GetVar(1))->GetRawAxis(twistSenseJoy.GetVar(2));

			if(setTwistSense)
			{
				zMultiplier = 1.0f + twistSense;
			}

			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone()*zMultiplier ? twist : 0;

			/*curveX = -driveX;//To fix the robot moving in the opposite direction
			curveY = -driveY;
			curveT = -twist;*/


			if(curveY < driveY)//I really should replace this with an array and a for loop or something
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

			xVel = -curveX;//setting values to correct orientation
			yVel = -curveY;
			if(-curveT > 0)//Should give twist more control
			{
				twistVel = -curveT - Spyder::GetDeadzone();
			}
			else if(-curveT < 0)
			{
				twistVel = -curveT + Spyder::GetDeadzone();
			}


			m_robotDrive->MecanumDrive_Cartesian(xVel, yVel, twistVel);//setting mecanum drive with curved values
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

