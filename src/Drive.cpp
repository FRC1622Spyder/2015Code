
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
	float rampVal;

	float xVel;
	float yVel;
	float twistVel;
	float curveX;
	float curveY;
	float curveT;

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
		Spyder::ConfigVar<float> accelRamp("driveAccelRampVal", 0.02);

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
		driveY = 0.0f;
		twist = 0.0f;
		curveX = 0.0f;
		curveY = 0.0f;
		curveT = 0.0f;

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
		Spyder::TwoIntConfig rightJoystick("rightJoyBind", 0, 0);
		Spyder::ConfigVar<uint32_t> yJoystick("yJoyBind",1);
		Spyder::ConfigVar<uint32_t> zJoystick("twistJoyBind", 3);
		Spyder::TwoIntConfig senseButton("driveTwistSensitivitySetButton",0, 2);
		Spyder::TwoIntConfig twistSenseJoy("twistSensitivitySliderBind", 0, 4);
		driveStick = Spyder::GetJoystick(rightJoystick.GetVar(1));
		driveStick->SetAxisChannel(Joystick::kTwistAxis, zJoystick.GetVal());

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
				if (autoRunTime > 3.5)//after 3.5 seconds of driving, stop (7 sec total)
				{
					frontLeftMotor->Set(0);
					backLeftMotor->Set(0);
					frontRightMotor->Set(0);
					backRightMotor->Set(0);
					++autoPhase;
					autoStart = curTime;
				}
				break;
			case 2:
				if (autoRunTime > 5)//after 5 seconds, drive back (12 sec total)
				{
					frontLeftMotor->Set(-0.5);
					backLeftMotor->Set(-0.5);
					frontRightMotor->Set(-0.5);
					backRightMotor->Set(-0.5);
					++autoPhase;
					autoStart = curTime;
				}
				break;
			case 3:
				if(autoRunTime > 1)//after 1 sec, stop (13 sec total)
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
			driveX = driveStick->GetRawAxis(rightJoystick.GetVar(2));
			driveY = driveStick->GetRawAxis(yJoystick.GetVal());//Setting axis of joystick
			twist = driveStick->GetTwist();

			/*std::cout<<frontLeftMotor->GetEncPosition()<<std::endl;
			std::cout<<backLeftMotor->GetEncPosition()<<std::endl;
			std::cout<<frontRightMotor->GetEncPosition()<<std::endl;
			std::cout<<backRightMotor->GetEncPosition()<<std::endl;*/

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

			curveX = -driveX;//To fix the robot moving in the opposite direction
			curveY = -driveY;
			curveT = -twist * zMultiplier;;


			if(yVel < curveY)//RAMP FUCKING WORKS
			{
				yVel += rampVal;
			}
			else if(yVel > curveY)
			{
				yVel -= rampVal;
			}
			else
			{
				yVel = curveY;
			}

			if(xVel < curveX)
			{
				xVel += rampVal;
			}
			else if(xVel > curveX)
			{
				xVel -= rampVal;
			}
			else
			{
				xVel = curveX;
			}

			if(twistVel < curveT)
			{
				twistVel += rampVal;
			}
			else if(twistVel > curveT)
			{
				twistVel -= rampVal;
			}
			else
			{
				twistVel = curveT;
			}

			/*xVel = -driveX;//setting values to correct orientation
			yVel = -driveY;
			twistVel = -twist * zMultiplier;
			if(-twist > 0)//Should give twist more control
			{
				twistVel = -twist - Spyder::GetDeadzone();
			}
			else if(-twist < 0)
			{
				twistVel = -twist + Spyder::GetDeadzone();
			}*/

			std::cout<<"BackRightMotorSetting = "<<backRightMotor->Get()<<std::endl;
			std::cout<<"FrontRightMotorSetting = "<<frontRightMotor->Get()<<std::endl;


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


