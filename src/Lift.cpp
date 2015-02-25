#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Lift : public Spyder::Subsystem
{
private:
	    bool lift1PosButton;//Lift Button setting
		bool lift2PosButton;
		bool lift3PosButton;
		bool tiltButton;//Tilt button setting
		bool manContButton;

		double lift1Pos;//Position setting
		double lift2Pos;
		double lift3Pos;
		double setDistance;

		double P;
		double I;
		double D;

		//Store current values to determine number of totes
		/*float toteCurrentOne;
		float toteCurrentTwo;
		float toteCurrentThree;
		float toteCurrentFour;
		float toteCurrentFive;
		float toteCurrentSix;*/

		double liftCurrent;//Monitor lift current draw to estimate number of totes
		int toteNumber;//Number of totes being carried

		CANTalon *liftMotor;//Motor Controller
		CANTalon *tiltMotor;
		PowerDistributionPanel *pdp;
		Joystick *manControl;//manual control
		Joystick *driveControl;
		Joystick *driveControl2;
		SmartDashboard *smartDashboard;

		unsigned char autoPhase;
		double autoStart;
		double pulsesPerDistance;//in inches
		double encoderStartTime;

public:
	Lift() : Spyder::Subsystem("Lift")
	{

	}

	virtual ~Lift()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		pulsesPerDistance = 1024/(2.873 * 3.14);//C = pitch diameter * pi

		Spyder::ConfigVar<double> firstLiftPosVal("liftPos1DistanceVal", 10);//Configure lift distances
		Spyder::ConfigVar<double> secondLiftPosVal("liftPos2DistanceVal", 20);//Height of tote = 12 inches
		Spyder::ConfigVar<double> thirdLiftPosVal("liftPos3DistanceVal", -20);
		//lift1Pos = firstLiftPosVal.GetVal();
		//lift2Pos = secondLiftPosVal.GetVal();
		//lift3Pos = thirdLiftPosVal.GetVal();

		lift1Pos = 10;
		lift2Pos = 20;
		lift3Pos = -20;

		Spyder::ConfigVar<double> P_Val("P_ValueForLiftPID", 0.5);//Set PID Values
		Spyder::ConfigVar<double> I_Val("I_ValueForLiftPID", 0);
		Spyder::ConfigVar<double> D_Val("D_ValueForLiftPID", 0);

		//P = P_Val.GetVal();
		//I = I_Val.GetVal();
		//D = D_Val.GetVal();

		Spyder::ConfigVar<int> liftMotorVal ("liftMotorCAN_id", 3);//Configure Lift Motor
		liftMotor = new CANTalon(liftMotorVal.GetVal());
		liftMotor->SetPosition(0);

		Spyder::ConfigVar<int> tiltMotorVal ("tiltMotorCAN_id",6);//Configure Tilt Motor
		tiltMotor = new CANTalon(tiltMotorVal.GetVal());

		driveControl = new Joystick(0);
		driveControl2 = new Joystick(2);

		pdp = new PowerDistributionPanel();//Configure PDP

		//Initialize current draw readings + values that need to be checked against readings
		/*Spyder::ConfigVar<float> oneToteCurVal("oneToteCurrentDraw", 1);//Configure current draws for number of totes being carried
		Spyder::ConfigVar<float> twoToteCurVal("twoToteCurrentDraw", 2);
		Spyder::ConfigVar<float> threeToteCurVal("threeToteCurrentDraw", 3);
		Spyder::ConfigVar<float> fourToteCurVal("fourToteCurrentDraw", 4);
		Spyder::ConfigVar<float> fiveToteCurVal("fiveToteCurrentDraw", 5);
		Spyder::ConfigVar<float> sixToteCurVal("sixToteCurrentDraw", 6);
		toteCurrentOne = oneToteCurVal.GetVal();
		toteCurrentTwo = twoToteCurVal.GetVal();
		toteCurrentThree = threeToteCurVal.GetVal();
		toteCurrentFour = fourToteCurVal.GetVal();
		toteCurrentFive = fiveToteCurVal.GetVal();
		toteCurrentSix = sixToteCurVal.GetVal();*/
		//liftControl = new PIDController(P_Val.GetVal(), I_Val.GetVal(), D_Val.GetVal(), liftEncoder, liftMotor);//Create PID system for liftmotor. Uses lift encoder values.

		struct timespec tp;

		switch(runmode)
		{
		case Spyder::M_AUTO:
			autoPhase = 0;
			clock_gettime(CLOCK_REALTIME, &tp);
			autoStart = (double) tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			clock_gettime(CLOCK_REALTIME, &tp);
			encoderStartTime = (double) tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			break;

		default:
			break;
		}
	}
	virtual void Periodic(Spyder::RunModes runmode)
	{
		Spyder::TwoIntConfig firstLiftPos("liftPos1ButtonVal", 1, 1);//Configure Lift Buttons (DOES NOT WORK)
		Spyder::TwoIntConfig secondLiftPos("liftPos2ButtonVal", 1, 4);
		Spyder::TwoIntConfig thirdLiftPos("liftPos3ButtonVal", 1, 3);
		lift1PosButton = Spyder::GetJoystick(firstLiftPos.GetVar(1))->GetRawButton(firstLiftPos.GetVar(2));
		lift2PosButton = Spyder::GetJoystick(secondLiftPos.GetVar(1))->GetRawButton(secondLiftPos.GetVar(2));
		lift3PosButton = Spyder::GetJoystick(thirdLiftPos.GetVar(1))->GetRawButton(thirdLiftPos.GetVar(2));
		//P = (driveControl2->GetRawAxis(3)+1)/20;
		//I = (driveControl->GetRawAxis(2)+1)/20;
		//D = (driveControl->GetRawAxis(4)+1)/20;

		setDistance = 0.0;
		//liftMotor->SetPID(P,I,D);

		Spyder::TwoIntConfig setManControl("setManControlButtonVal", 1, 7);
		manContButton = Spyder::GetJoystick(setManControl.GetVar(1))->GetRawButton(setManControl.GetVar(2));

		Spyder::TwoIntConfig tiltPos("tiltButtonVal", 1, 2);//Configure Tilt Buttons
		tiltButton = Spyder::GetJoystick(tiltPos.GetVar(1))->GetRawButton(tiltPos.GetVar(2));

		//bool encoderTest;
		//bool tilt = false;
		//int encoderTestStart = 0;

		float manualControl;
		float manualTilt;
		//liftCurrent = pdp->GetCurrent(liftMotor->GetChannel());//Getting current draw of motor

		Spyder::TwoIntConfig manualControlJoy("liftManualControlJoystickVal", 1, 3);//Setting manual control to joystick
		manControl = Spyder::GetJoystick(manualControlJoy.GetVar(1));

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
				if(autoRunTime  > 1.5)//wait for claw
				{
					++autoPhase;
					autoStart = curTime;
				}
				break;
			case 1:				liftMotor->SetControlMode(CANSpeedController::kPosition);
				if(autoRunTime < 1)//pick box up
				{
					liftMotor->Set(1);
				}
				else//hold after 1.5 seconds
				{
					liftMotor->Set(0);
					autoPhase++;
					autoStart = curTime;
				}
				break;
			case 2:
			{
				if(autoRunTime > 10 && autoRunTime < 11)//drop box down after 10 seconds
				{
					liftMotor->Set(-1);
				}
				else//wait
				{
					liftMotor->Set(0);
				}
				break;
			}
			default:
				break;
			}
			break;
		}
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
		{
			smartDashboard->PutNumber("Lift P Val", P);
			smartDashboard->PutNumber("Lift I Val", I);
			smartDashboard->PutNumber("Lift D Val", D);
			smartDashboard->PutNumber("LiftSetDistance", setDistance);
			smartDashboard->PutNumber("liftEncoderPosition", liftMotor->GetEncPosition());

			/*struct timespec tp;
			clock_gettime(CLOCK_REALTIME, &tp);
			double curTime = (double)tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			double encoderTestTime = curTime - encoderStartTime;

			if(encoderTestTime <= 0.5)
			{
				encoderTestStart = liftMotor->GetEncPosition();
			}
			else if(encoderTestTime > 3)
			{
				if(encoderTestStart != liftMotor->GetEncPosition())
				{
					encoderTest = true;
					std::cout<<"Encodertest = "<<encoderTest<<std::endl;
				}
				else
				{
					encoderTest = false;
					std::cout<<"Encodertest = "<<encoderTest<<std::endl;
				}
				encoderStartTime = curTime;
			}*/


			manualControl = manControl->GetRawAxis(manualControlJoy.GetVar(2));
			manualTilt = manControl->GetRawAxis(manualControlJoy.GetVar(1));
			manualControl = fabs(manualControl) > Spyder::GetDeadzone() ? manualControl : 0;//Manual Control deadzone
			manualTilt = fabs(manualTilt) > Spyder::GetDeadzone() ? manualTilt : 0;

			tiltMotor->Set(manualTilt/3);
			liftMotor->Set(manualControl);
			/*if(!manContButton)
			{
				liftMotor->SetControlMode(CANSpeedController::kPosition);
				if(lift1PosButton)//Basic PID control for lift. Does not account for different values of totes.
				{
					liftMotor->Set(lift1Pos);
					std::cout<<"LiftMotorGet() = "<<liftMotor->Get()<<std::endl;
					setDistance = lift1Pos;
				}
				if(lift2PosButton)//Basic PID control for lift. Does not account for different values of totes.
				{
					liftMotor->Set(lift2Pos);
					std::cout<<"LiftMotorGet() = "<<liftMotor->Get()<<std::endl;
					setDistance = lift2Pos;
				}
				if(lift3PosButton)//Basic PID control for lift. Does not account for different values of totes.
				{
					liftMotor->Set(lift3Pos);
					std::cout<<"LiftMotorGet() = "<<liftMotor->Get()<<std::endl;
					setDistance = lift3Pos;
				}
			}
			else
			{
				liftMotor->SetControlMode(CANSpeedController::kPercentVbus);
				liftMotor->Set(manualControl);
			}*/

			//reset lift encoder when it hits home
			/*if(liftMotor->isFwdLimitSwitchClosed())
			{
				liftMotor->SetPosition(0);
			}*/

			/*if(encoderTestTime <= 0.5)
			{
				encoderTestStart = liftEncoder->Get();
			}
			else if(encoderTestTime > 3)
			{
				if(encoderTestStart != liftEncoder->Get())
				{
					encoderTest = true;
				}
				else
				{
					encoderTest = false;
				}
				encoderTestStart = curTime;
			}

			if(tiltButton)//Set tilt to move 1/2
			{
				tilt = true;
				tiltEncoder->Reset();
			}
			if(tilt && tiltEncoder->GetDistance() < 1/2)
			{
				tiltMotor->Set(0.2);
			}
			else if(tilt && tiltEncoder->GetDistance() >= 1/2)
			{
				tiltMotor->Set(0);
				tilt = false;
			}

			//Establish number of totes with current draw
			/*if(liftCurrent <= toteCurrentOne)//totes = 0
			{
				//Set new PID values
			}
			else if(toteCurrentOne < liftCurrent && liftCurrent < toteCurrentTwo)//totes = 1
			{
			}
			else if(toteCurrentTwo < liftCurrent && liftCurrent < toteCurrentThree)//totes = 2
			{
			}
			else if(toteCurrentThree < liftCurrent && liftCurrent < toteCurrentFour)//totes = 3
			{
			}
			else if(toteCurrentFour < liftCurrent && liftCurrent < toteCurrentFive)//totes = 4
			{
			}
			else if(toteCurrentFive < liftCurrent && liftCurrent < toteCurrentSix)//totes = 5
			{
			}
			else //totes = 6
			{
			}*/
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
Lift lift;

