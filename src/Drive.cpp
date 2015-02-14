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

	/*float absVal_x; // For when we use a custom mechanum drive
	float absVal_y;
	float absVal_z;
	float axisTotVal;
	float xWeight;
	float yWeight;
	float zWeight;

	float leftTopSpeed;
	float leftBackSpeed;
	float rightTopSpeed;
	flaot rightBackSpeed;*/

	float driveX;
	float curveX;
	float driveY;
	float curveY;
	float twist;
	float curveZ;
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

		/*absVal_x = 0.0f;
		absVal_y = 0.0f;
		absVal_z = 0.0f;
		axisTotVal = 0.0f;
		xWeight = 0.0f;
		yWeight = 0.0f;
		zWeight = 0.0f;*/

		driveX = 0.0f;//Store input of joystick to set speed of motors
		curveX = 0.0f;//speed setting after curve
		driveY = 0.0f;
		curveY = 0.0f;
		twist = 0.0f;//twist of joystick
		curveT = 0.0f;

		//m_robotDrive->SetSafetyEnabled(false);
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

			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;

			curveX = -driveX;//To fix the robot moving in the opposite direction
			curveY = -driveY;
			curveT = -twist;

			/*if(driveX < 0)
			{
				absVal_x = -driveX;
			}
			else
			{
				absVal_x = driveX;
			}
			if(driveY < 0)
			{
				absVal_y = -driveY;
			}
			else
			{
				absVal_y = driveY;
			}
			if(twist < 0)
			{
				absVal_z = -twist;
			}
			else
			{
				absVal_z = twist;
			}

			absVal_x + absVal_y + absVal_z = axisTotVal;

			absVal_x/axisTotVal = xWeight;
			absVal_y/axisTotVal = yWeight;
			absVal_z/axisTotVal = zWeight;

			frontLeftMotor->Set(driveY*yWeight - driveX*xWeight);
			backLeftMotor->Set(driveY*yWeight + driveX*xWeight);
			frontRightMotor->Set(driveY*yWeight + driveX*xWeight);
			backRightMotor->Set(driveY*yWeight - driveX*xWeight);
			*/

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

			if(curveT < twist)
			{
				curveT += rampVal;
			}
			else if(curveT > twist)
			{
				curveT -= rampVal;
			}*/

			//curveZ = -curveT;



			m_robotDrive->MecanumDrive_Cartesian(curveX, curveY, curveT);//setting mecanum drive with curved values
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

