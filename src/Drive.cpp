#include "Subsystem.h"
#include "WPIObjMgr.h"
#include "Config.h"
#include "Console.h"
#include <iostream>
#include <cmath>

class Drive : public Spyder::Subsystem
{
	
	private:
		Spyder::TwoIntConfig leftJoystick;
		Spyder::TwoIntConfig rightJoystick;

		Spyder::ConfigVar<uint32_t> leftTopMotor;
		Spyder::ConfigVar<uint32_t> leftBottomMotor;
		Spyder::ConfigVar<uint32_t> rightTopMotor;
		Spyder::ConfigVar<uint32_t> rightBottomMotor;
		
		Spyder::ConfigVar<bool> motorReverse;
		Spyder::ConfigVar<unsigned int> teleopPhase;
		Spyder::ConfigVar<unsigned int> twistAxis;
		unsigned char autoPhase;
		RobotDrive *m_RobotDrive;
		public:
		
		Drive() : Spyder::Subsystem("Drive"), leftJoystick("bind_leftDriveJoy", 1, 1), //leftJoystick("bind_leftDrive", 1, 2),
		rightJoystick("bind_rightDriveJoy", 2, 1), leftTopMotor("leftTopMotorPort", 0),
		leftBottomMotor("leftBottomMotorPort", 1), rightTopMotor("rightTopMotorPort", 2),
		rightBottomMotor("rightBottomMotorPort", 3), motorReverse("invertDriveButton", 1),
		teleopPhase("driveType", 0), twistAxis("rightJoyTwistAxis", 3), autoPhase(0), m_RobotDrive()
		{
		}
		
		virtual ~Drive()
		{
		}
		
		virtual void Init(Spyder::RunModes runmode)
		{
			m_robotDrive = new RobotDrive(leftTopMotor.GetVal(), leftBottomMotor.GetVal(), rightTopMotor.GetVal(), rightBottomMotor.GetVal())
			switch(runmode)
			{
				case Spyder::M_AUTO:
					autoPhase = 0;
					break;
				default:
					;
			}
		}
		
		virtual void Periodic(Spyder::RunModes runmode)
		{
			Joystick *leftJoy = Spyder::GetJoystick(leftJoystick.GetVar(1));
			Joystick *rightJoy = Spyder::GetJoystick(rightJoystick.GetVar(1));

			switch(runmode){
				case Spyder::M_AUTO:
					switch(autoPhase)
					{
					case 0:
					case 1:
						//Spyder::GetVictor(leftMotor.GetVal())->Set(1);
						break;
					case 2:
						break;
					}
					break;
				
				case Spyder::M_DISABLED:
					break;
					
				case Spyder::M_TELEOP:
					left_x = leftJoy->GetRawAxis(leftJoystick.GetVar(1));
					left_y = leftJoy->GetRawAxis(leftJoystick.GetVar(2));
					right_x = rightJoy->GetRawAxis(rightJoystick.GetVar(1));
					right_y = rightJoy->GetRawAxis(rightJoystick.GetVar(2));
					rightTwist = rightJoy->SetAxisChannel(Joystick::kTwistAxis, twistAxis.GetVal());
					left_x = fabs(left_x) > Spyder::GetDeadzone() ? left_x : 0;
					left_y = fabs(left_y) > Spyder::GetDeadzone() ? left_y : 0;
					right_x = fabs(right_x) > Spyder::GetDeadzone() ? right_x : 0;
					right_y = fabs(right_y) > Spyder::GetDeadzone() ? right_y : 0;
					rightTwist = fabs(rightTwist) > Spyder::GetDeadzone() ? rightTwist : 0;
					switch(teleopPhase.GetVal())
					{
					case 0://parade drive
						left_y /= 2;
						right_y /= 2;
						Spyder::GetVictor(leftTopMotor.GetVal())->Set(left_y);
						Spyder::GetVictor(leftBottomMotor.GetVal())->Set(left_y);
						Spyder::GetVictor(rightBottomMotor.GetVal())->Set(right_y);
						Spyder::GetVictor(rightTopMotor.GetVal())->Set(right_y);
						break;
					}
					case 1: //mecanum drive (competition)
					{
						m_robotDrive->MecanumDrive_Cartesian(right_x, right_y, rightTwist);
					}

					
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
