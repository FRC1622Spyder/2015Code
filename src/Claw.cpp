#include "subsystem.h"
#include "Config.h"
#include "WPIObjMgr.h"
#include "WPILib.h"
#include <iostream>
#include <cmath>

class Claw : public Spyder::Subsystem
{
private:
	bool openClawButton;
	bool closeClawButton;
	DoubleSolenoid *clawSol;
	unsigned char autoPhase;
	double autoStart;
public:
	Claw() : Spyder::Subsystem("Claw")
	{
	}

	virtual ~Claw()
	{
	}

	virtual void Init(Spyder::RunModes runmode)
	{
		openClawButton = false;
		closeClawButton = false;
		Spyder::ConfigVar<uint8_t> solModule("clawSolModuleNumber", 7);
		Spyder::TwoIntConfig solPorts("clawSolPorts", 0, 1);
		clawSol = new DoubleSolenoid (solModule.GetVal(), solPorts.GetVar(1), solPorts.GetVar(2));

		struct timespec tp;
		switch(runmode)
		{
		case Spyder::M_AUTO:
			autoPhase = 0;
			clock_gettime(CLOCK_REALTIME, &tp);
			autoStart = (double)tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
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
		Spyder::TwoIntConfig openClaw ("openClawButtonVal", 1, 5);//Initialize the claw ports
		Spyder::TwoIntConfig closeClaw ("closeClawButtonVal", 1, 6);

		switch(runmode)
		{
		case Spyder::M_AUTO:
		{/*
			struct timespec tp;
			clock_gettime(CLOCK_REALTIME, &tp);
			double curTime = (double) tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			double autoRunTime = curTime - autoStart;

			switch(autoPhase)
			{
			case 0:
				if(autoRunTime < 1)//Grab tote
				{
					clawSol->Set(DoubleSolenoid::kReverse);
				}
				else
				{
					clawSol->Set(DoubleSolenoid::kOff);
				}

				if(autoRunTime > 20)//Wait 20 seconds
				{
					autoPhase++;
					autoStart = curTime;
				}
				break;
			case 1:
				if(autoRunTime < 1)//release tote
				{
					clawSol->Set(DoubleSolenoid::kForward);
				}
				else
				{
					clawSol->Set(DoubleSolenoid::kOff);
				}
				break;
			default:
				break;
			}
*/
			break;
		}
		case Spyder::M_DISABLED:
			break;
		case Spyder::M_TELEOP:
			openClawButton = Spyder::GetJoystick(openClaw.GetVar(1))->GetRawButton(openClaw.GetVar(2));//Get Button Vals
			closeClawButton = Spyder::GetJoystick(closeClaw.GetVar(1))->GetRawButton(closeClaw.GetVar(2));

			if(openClawButton)//Open Claw
			{
				clawSol->Set(DoubleSolenoid::kForward);
				std::cout<<"OPEN_CLAW"<<std::endl;
			}
			else if(closeClawButton)//Close Claw
			{
				clawSol->Set(DoubleSolenoid::kReverse);
				std::cout<<"CLOSE_CLAW"<<std::endl;
			}
			else
			{
				clawSol->Set(DoubleSolenoid::kOff);
			}
			break;
		default:
			break;
		}
	}

	virtual void RobotInit()
	{
	}
};
Claw claw;
