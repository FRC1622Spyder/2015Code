#include "WPILib.h"
#include "Config.h"
#include "subsystem.h"
#include "RGBStrip.h"
#include <fstream>

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	//CameraServer *cameraServer;
	//SmartDashboard *smartDashboard;
	Spyder::RGBStrip *ledStrip1, *ledStrip2;
	unsigned int usPeriodCounter;
public:
	void RobotInit()
	{
		//uncommented these lines
		Spyder::ConfigVar<uint32_t> rChannel1("redLEDChannel1", 1);
		Spyder::ConfigVar<uint32_t> gChannel1("greenLEDChannel1", 2);
		Spyder::ConfigVar<uint32_t> bChannel1("blueLEDChannel1", 0);
		Spyder::ConfigVar<uint32_t> rChannel2("redLEDChannel2", 4);
		Spyder::ConfigVar<uint32_t> gChannel2("greenLEDChannel2", 5);
		Spyder::ConfigVar<uint32_t> bChannel2("blueLEDChannel2", 3);
		ledStrip1 = new Spyder::RGBStrip(rChannel1.GetVal(), gChannel1.GetVal(), bChannel1.GetVal());
		ledStrip2 = new Spyder::RGBStrip(rChannel2.GetVal(), gChannel2.GetVal(), bChannel2.GetVal());

		ledStrip1->SetColor(0, 0, 2000);
		ledStrip2->SetColor(0, 2000, 0);

		lw = LiveWindow::GetInstance();
		//cameraServer = CameraServer::GetInstance(); //camera code
		//cameraServer->StartAutomaticCapture();


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
		//while in autonomous, showing yellow lights like the field does
		ledStrip1->SetColor(2000,1000,0);
		ledStrip2->SetColor(2000,1000,0);
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
		DriverStation *tds = DriverStation::GetInstance();
		DriverStation::Alliance tColor = tds->GetAlliance();

		switch( tColor ){
		case DriverStation::Alliance::kBlue:
			//We're on the blue alliance
			ledStrip1->SetColor( 0, 0, 2000);
			ledStrip2->SetColor( 0, 0, 2000 );
			break;
		case DriverStation::Alliance::kRed:
				//We're on the red alliance
				ledStrip1->SetColor( 2000, 0, 0 );
				ledStrip2->SetColor( 2000, 0, 0 );
				break;
		default:
			//We're neither blue nor red. This should never happen, so show green because Poway
			ledStrip1->SetColor( 0, 2000, 0 );
			ledStrip2->SetColor( 0, 2000, 0 );
		}

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
