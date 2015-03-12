#ifndef SRC_CLAW_H_
#define SRC_CLAW_H_

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
	bool clawPosition;
	DoubleSolenoid *clawSol;
	unsigned char autoPhase;
	double autoStart;
	int init;
public:
	bool isClawClosed()
	{
		return clawPosition;
	}
	Claw() : Spyder::Subsystem("Claw"),init(0)
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
		if(init == 0)
		{
			clawSol = new DoubleSolenoid (solModule.GetVal(), solPorts.GetVar(1), solPorts.GetVar(2));
		}
		init++;

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
		{
			struct timespec tp;
			clock_gettime(CLOCK_REALTIME, &tp);
			double curTime = (double) tp.tv_sec + double(double(tp.tv_nsec)*1e-9);
			double autoRunTime = curTime - autoStart;

			switch(autoPhase)
			{
			//K'S AUTO 1
			/*case 0:
			  if(autoRunTime > 0.5) //close after 0.5
			  {
			  clawSol -> Set(DoubleSolenoid::kReverse);
			  autoPhase++;
			  autoStart = curTime;
			  }
			  break;

			  case1:
			  if(autoRunTime < 3.5) //after 3.5 open
			  {
			  clawSol -> Set(DoubleSolenoid::kForward);
			  autoPhase++;
			  autoStart = curTime;
			  }
			  break;*/

			//K'S AUTO 2

			/*case 0:
				if(autoRunTime < 1)//start by
				{
					clawSol->Set(DoubleSolenoid::kForward);
				}
				else
				{
					clawSol->Set(DoubleSolenoid::kOff);
					autoPhase++;
					autoStart = curTime;
				}
				break;
			case 1:
				if(autoRunTime > 3.5)//Grab tote
				{
					clawSol->Set(DoubleSolenoid::kReverse);
				}
				else
				{
					clawSol->Set(DoubleSolenoid::kOff);
				}

				if(autoRunTime > 7.5)//Wait 7.5 seconds
				{
					autoPhase++;
					autoStart = curTime;
				}
				break;
			case 2:
				if(autoRunTime < 1)//release tote
				{
					clawSol->Set(DoubleSolenoid::kForward);
				}
				else
				{
					clawSol->Set(DoubleSolenoid::kOff);
				}
				break;*/
			default:
				break;
			}
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
				clawPosition = false;
			}
			else if(closeClawButton)//Close Claw
			{
				clawSol->Set(DoubleSolenoid::kReverse);
				std::cout<<"CLOSE_CLAW"<<std::endl;
				clawPosition = true;
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


#endif /* SRC_CLAW_H_ */
