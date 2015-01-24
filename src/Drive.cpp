#include "Subsystem.h"
#include "WPIObjMgr.h"
#include "Config.h"
#include "Console.h"
#include <iostream>
#include <cmath>

class Drive : public Spyder::Subsystem
{
	
	private:
		//Spyder::TwoIntConfig leftJoystick;
		
		//Spyder::ConfigVar<bool> leftMotorInv;
		//unsigned char autoPhase;
	public:
		
		Drive() : Spyder::Subsystem("Drive") //leftJoystick("bind_leftDrive", 1, 2),
		{
		}
		
		virtual ~Drive()
		{
		}
		
		virtual void Init(Spyder::RunModes runmode)
		{
			switch(runmode)
			{
				case Spyder::M_AUTO:
					//autophase = 0;
					break;
				default:
					;
			}
		}
		
		virtual void Periodic(Spyder::RunModes runmode)
		{
			//Joystick *leftJoy = Spyder::GetJoystick(leftJoystick.GetVar(1));
			switch(runmode){
				case Spyder::M_AUTO:
					/*switch(autoPhase)
					{
					case 0:
					case 1:
						//Spyder::GetVictor(leftMotor.GetVal())->Set(1);
						break;
					case 2:
						break;
					}
					break;*/
				
				case Spyder::M_DISABLED:
					break;
					
				case Spyder::M_TELEOP:
					//left = leftJoy->GetRawAxis(leftJoystick.GetVar(2));
					//left = fabs(left) > Spyder::GetDeadzone() ? left : 0;

					
					//if(Spyder::GetJoystick(halfSpeed.GetVar(1))->GetRawButton(halfSpeed.GetVar(2)))
					
					//Spyder::GetVictor(leftMotor.GetVal())->Set(left);
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
