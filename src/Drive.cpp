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
	float driveY;
	float twist;

	float xVel;
	float yVel;
	float twistVel;

	double autoStart;
	unsigned char autoPhase;

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
		Spyder::ConfigVar<int> leftFrontCAN("driveLeftFrontCAN_id", 5);
		Spyder::ConfigVar<int> rightFrontCAN("driveRightFrontCAN_id", 2);
		Spyder::ConfigVar<int> leftBackCAN("driveLeftBackCAN_id", 4);
		Spyder::ConfigVar<int> rightBackCAN("driveRightBackCAN_id", 1);


		frontLeftMotor = new CANTalon(leftFrontCAN.GetVal());
		frontRightMotor = new CANTalon(rightFrontCAN.GetVal());
		backLeftMotor = new CANTalon(leftBackCAN.GetVal());
		backRightMotor = new CANTalon(rightBackCAN.GetVal());

		m_robotDrive = new RobotDrive(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);//Configure mecanum drive

		m_robotDrive->SetExpiration(0.1);
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		twistSense = 0.0f;
		zMultiplier = 1.0f;
		setTwistSense = false;

		driveX = 0.0f;//Store input of joystick to set speed of motors
		driveY = 0.0f;
		twist = 0.0f;

		xVel = 0.0f;
		yVel = 0.0f;
		twistVel = 0.0f;

		m_robotDrive->SetSafetyEnabled(false);

		struct timespec tp;
		switch(runmode)
		{
		case Spyder::M_AUTO:
			autoPhase = 0;
			clock_gettime(CLOCK_REALTIME, &tp);
			autoStart = (double)tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
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
		driveStick->SetAxisChannel(Joystick::kTwistAxis, 3);

		switch(runmode)
		{
		case Spyder::M_AUTO:
		{
			struct timespec tp;
			clock_gettime(CLOCK_REALTIME, &tp);
			double curTime = (double)tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			double autoRunTime = curTime - autoStart;

			switch(autoPhase)
			{
			case 0:
				if(autoRunTime >= 3.5)//after 3.5 seconds, drive forward
				{
					frontLeftMotor->Set(-0.5);
					backLeftMotor->Set(-0.5);
					frontRightMotor->Set(0.5);
					backRightMotor->Set(0.5);
					++autoPhase;
					autoStart = curTime;
				}
				break;
			case 1:
				if (autoRunTime > 7)//after 3.5 seconds of driving, stop
				{
					frontLeftMotor->Set(0);
					backLeftMotor->Set(0);
					frontRightMotor->Set(0);
					backRightMotor->Set(0);
				}
				break;
			default:
				break;
			}
			break;
		}

		case Spyder::M_DISABLED:
			break;

		case Spyder::M_TELEOP:
		{
			driveX = driveStick->GetRawAxis(rightJoystick.GetVar(1));
			driveY = driveStick->GetRawAxis(rightJoystick.GetVar(2));//Setting axis of joystick
			twist = driveStick->GetTwist();

			setTwistSense = Spyder::GetJoystick(senseButton.GetVar(1))->GetRawButton(senseButton.GetVar(2));//Twist sensitivity settings!
			twistSense = Spyder::GetJoystick(twistSenseJoy.GetVar(1))->GetRawAxis(twistSenseJoy.GetVar(2));
			//twistSense = driveStick->GetRawAxis(3);

			if(setTwistSense)
			{
				zMultiplier = (twistSense+1)/2;
			}


			driveX = fabs(driveX) > Spyder::GetDeadzone() ? driveX : 0;//Set proper deadzone;
			driveY = fabs(driveY) > Spyder::GetDeadzone() ? driveY : 0;
			twist = fabs(twist) > Spyder::GetDeadzone() ? twist : 0;

			/*curveX = -driveX;//To fix the robot moving in the opposite direction
			curveY = -driveY;
			curveT = -twist;*/


			/*if(curveY < driveY -.05)//I really should replace this with an array and a for loop or something
			{
				curveY += rampVal;
			}
			else if(curveY > driveY +.05)
			{
				curveY -= rampVal;
			}
			else
			{
				curveY = driveY;
			}

			if(curveX < driveX - .05)
			{
				curveX += rampVal;
			}
			else if(curveX > driveX + .05)
			{
				curveX -= rampVal;
			}
			else
			{
				curveX = driveX;
			}

			if(curveT < twist - .05)
			{
				curveT += rampVal;
			}
			else if(curveT > twist + .05)
			{
				curveT -= rampVal;
			}
			else
			{
				curveT = twist;
			}*/

			xVel = -driveX;//setting values to correct orientation
			yVel = -driveY;
			twistVel = -twist * zMultiplier;
			/*if(-twist > 0)//Should give twist more control
			{
				twistVel = -twist - Spyder::GetDeadzone();
			}
			else if(-twist < 0)
			{
				twistVel = -twist + Spyder::GetDeadzone();
			}*/


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

