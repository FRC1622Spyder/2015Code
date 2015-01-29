#include "WPILib.h"
#include "subsystem.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	unsigned int usPeriodCounter;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->RobotInit();
		}
	}

	void AutonomousInit()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Init(Spyder::M_AUTO);
		}

	}

	void AutonomousPeriodic()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Periodic(Spyder::M_AUTO);
		}

	}

	void TeleopInit()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Init(Spyder::M_TELEOP);
		}

	}

	void TeleopPeriodic()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Periodic(Spyder::M_TELEOP);
		}

	}

	void TestPeriodic()
	{
		lw->Run();
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for(size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Periodic(Spyder::M_TEST);
		}
	}
};

START_ROBOT_CLASS(Robot);
