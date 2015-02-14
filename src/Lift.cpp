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

		double lift1Pos;//Position setting
		double lift2Pos;
		double lift3Pos;

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
		PIDController *liftControl;//PID Controller
		//Encoder *liftEncoder;//Lift Encoder
		//Encoder *tiltEncoder;//Tilt Encoder
		PowerDistributionPanel *pdp;
		Joystick *manControl;//manual control
		DigitalInput *liftHome;
public:
	Lift() : Spyder::Subsystem("Lift")
	{

	}

	virtual ~Lift()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		Spyder::TwoIntConfig firstLiftPos("liftPos1ButtonVal", 2, 1);//Configure Lift Buttons
		Spyder::TwoIntConfig secondLiftPos("liftPos2ButtonVal", 2, 4);
		Spyder::TwoIntConfig thirdLiftPos("liftPos3ButtonVal", 2, 3);
		lift1PosButton = Spyder::GetJoystick(firstLiftPos.GetVar(1))->GetRawButton(firstLiftPos.GetVar(2));
		lift2PosButton = Spyder::GetJoystick(secondLiftPos.GetVar(1))->GetRawButton(secondLiftPos.GetVar(2));
		lift3PosButton = Spyder::GetJoystick(thirdLiftPos.GetVar(1))->GetRawButton(thirdLiftPos.GetVar(2));

		Spyder::TwoIntConfig tiltPos("tiltButtonVal", 2, 2);//Configure Tilt Buttons
		tiltButton = Spyder::GetJoystick(tiltPos.GetVar(1))->GetRawButton(tiltPos.GetVar(2));

		Spyder::ConfigVar<double> firstLiftPosVal("liftPos1DistanceVal", 10);//Configure lift distances
		Spyder::ConfigVar<double> secondLiftPosVal("liftPos2DistanceVal", 20);
		Spyder::ConfigVar<double> thirdLiftPosVal("liftPos3DistanceVal", 30);
		lift1Pos = firstLiftPosVal.GetVal();
		lift2Pos = secondLiftPosVal.GetVal();
		lift3Pos = thirdLiftPosVal.GetVal();

		Spyder::ConfigVar<float> P_Val("P_ValueForLiftPID", 0.1);//Set PID Values
		Spyder::ConfigVar<float> I_Val("I_ValueForLiftPID", 0.001);
		Spyder::ConfigVar<float> D_Val("D_ValueForLiftPID", 0.0);

		/*Spyder::TwoIntConfig liftEncoderPorts("liftEncoderInputPortVals", 0, 1);//Configure Lift Encoder
		Spyder::ConfigVar<bool> liftInvertEncoder("invertLiftEncoder", false);
		liftEncoder = new Encoder(liftEncoderPorts.GetVar(1), liftEncoderPorts.GetVar(2), liftInvertEncoder.GetVal());

		Spyder::TwoIntConfig tiltEncoderPorts("tiltEncoderInputPortVals", 2, 3);//Configure tilt encoder
		Spyder::ConfigVar<bool> tiltInvertEncoder("invertTiltEncoder", false);
		tiltEncoder = new Encoder(tiltEncoderPorts.GetVar(1), tiltEncoderPorts.GetVar(2), tiltInvertEncoder.GetVal());
		tiltEncoder->SetDistancePerPulse(1/1024);*/

		Spyder::ConfigVar<int> liftMotorVal ("liftMotorCAN_id", 6);//Configure Lift Motor
		liftMotor = new CANTalon(liftMotorVal.GetVal());

		Spyder::ConfigVar<int> tiltMotorVal ("tiltMotorCAN_id",3);//Configure Tilt Motor
		tiltMotor = new CANTalon(tiltMotorVal.GetVal());

		Spyder::ConfigVar<uint32_t> liftHomePort("liftHomeLimitPort", 5);
		liftHome = new DigitalInput(liftHomePort.GetVal());

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

		//struct timespec tp;
		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			break;

		default:
			break;
		}
	}
	virtual void Periodic(Spyder::RunModes runmode)
	{
		//bool encoderTest;
		//bool tilt = false;
		//int encoderTestStart = 0;
		float manualControl;
		float manualTilt;
		/*struct timespec tp;//Time stuff is used to ensure encoder is working
		clock_gettime(CLOCK_REALTIME, &tp);
		double curTime = (double)tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
		double encoderTestTime = curTime - encoderTestStart;*/
		//liftControl->Enable();
		//liftCurrent = pdp->GetCurrent(liftMotor->GetChannel());//Getting current draw of motor

		Spyder::TwoIntConfig manualControlJoy("liftManualControlJoystickVal", 1, 3);//Setting manual control to joystick
		manControl = Spyder::GetJoystick(manualControlJoy.GetVar(1));


		switch(runmode)
		{
		case Spyder::M_AUTO:
			break;
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			manualControl = manControl->GetRawAxis(manualControlJoy.GetVar(2));
			manualTilt = manControl->GetRawAxis(manualControlJoy.GetVar(1));
			manualControl = fabs(manualControl) > Spyder::GetDeadzone() ? manualControl : 0;//Manual Control deadzone
			manualTilt = fabs(manualTilt) > Spyder::GetDeadzone() ? manualTilt : 0;

			liftMotor->Set(manualControl);
			tiltMotor->Set(manualTilt/2);

			//reset lift encoder when it hits home
			if(liftHome->Get())
			{
				//liftEncoder->Reset();
			}
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
			else//totes = 6
			{
			}
			if(encoderTest)//If encoder is good, use PID
			{
				if(lift1PosButton)//Basic PID control for lift. Does not account for different values of totes.
				{
					liftControl->SetSetpoint(lift1Pos);
				}
				if(lift2PosButton)
				{
					liftControl->SetSetpoint(lift2Pos);
				}
				if(lift3PosButton)
				{
					liftControl->SetSetpoint(lift3Pos);
				}
			}
			else//manual control if encoder doesn't work
			{
				liftMotor->Set(manualControl);
				/*if(lift1PosButton)                            //backup for the PID Controller
				{
					if(liftEncoder->GetDistance() == lift1Pos)
					{
						liftMotor->Set(0);
					}
					if(liftEncoder->GetDistance() > lift1Pos)
					{
						liftMotor->Set(-0.2);
					}
					if(liftEncoder->GetDistance() < lift1Pos)
					{
						liftMotor->Set(0.2);
					}
				}
				if(lift2PosButton)
				{
					if(liftEncoder->GetDistance() == lift2Pos)
					{
						liftMotor->Set(0);
					}
					if(liftEncoder->GetDistance() < lift2Pos)
					{
						liftMotor->Set(0.2);
					}
					if(liftEncoder->GetDistance() > lift2Pos)
					{
						liftMotor->Set(-0.2);
					}
				}
				if(lift3PosButton)
				{
					if(liftEncoder->GetDistance() == lift3Pos)
					{
						liftMotor->Set(0);
					}
					if(liftEncoder->GetDistance() < lift3Pos)
					{
						liftMotor->Set(0.2);
					}
					if(liftEncoder->GetDistance() > lift3Pos)
					{
						liftMotor->Set(-0.2);
					}
				}

			}*/
			break;
		default:
			break;
		}
	}
	virtual void RobotInit()
	{
	}
};
Lift lift;

