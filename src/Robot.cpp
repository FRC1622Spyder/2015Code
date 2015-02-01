#include "WPILib.h"
#include "Config.h"
#include "subsystem.h"
#include <fstream>

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	unsigned int usPeriodCounter;
public:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		std::fstream file;
		file.open("/etc/SpyderConfig.cfg", std::ios_base::in);
		if(file.is_open())
		{
			Spyder::ConfigVarBase::ReadConfigFile(file);
			file.close();
		}

		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->RobotInit();
		}
	}

	void AutonomousInit()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Init(Spyder::M_AUTO);
		}

	}

	void AutonomousPeriodic()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++) {
			if (usPeriodCounter % subsystems[i]->GetPeriod() == 0)
			{
				subsystems[i]->Periodic(Spyder::M_AUTO);
			}
		}
		++usPeriodCounter;
	}

	void TeleopInit()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++)
		{
			subsystems[i]->Init(Spyder::M_TELEOP);
		}

	}

	void TeleopPeriodic()
	{
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++) {
			if (usPeriodCounter % subsystems[i]->GetPeriod() == 0)
			{
				subsystems[i]->Periodic(Spyder::M_TELEOP);
			}
		}
		++usPeriodCounter;
	}

	void TestPeriodic()
	{
		lw->Run();
		std::vector<Spyder::Subsystem*> subsystems = Spyder::SubsystemMgr::GetSingleton()->GetSubsystems();
		for (size_t i = 0; i < subsystems.size(); i++) {
			if (usPeriodCounter % subsystems[i]->GetPeriod() == 0)
			{
				subsystems[i]->Periodic(Spyder::M_TEST);
			}
		}
		++usPeriodCounter;
	}
};

int main()
	{
		int halRes = HALInitialize();
		if (!halRes){
			std::cerr<<"FATAL ERROR: HAL could not be initialized"<<std::endl;return -1;
		}
		HALReport(HALUsageReporting::kResourceType_Language, HALUsageReporting::kLanguage_CPlusPlus);
		Robot *robot = new Robot();
		RobotBase::robotSetup(robot);
		return 0;
	}
