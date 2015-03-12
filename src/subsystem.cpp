#include "subsystem.h"
#include <iostream>

namespace Spyder
{
	SubsystemMgr* SubsystemMgr::GetSingleton()
	{
		static SubsystemMgr* s_smgr = new SubsystemMgr;
		return s_smgr;
	}

	void SubsystemMgr::RegisterSubsystem(const std::string &strName, Subsystem* ptr)
	{
		m_subsystems.resize(m_subsystems.size() +1);
		m_subsystems[m_subsystems.size()-1] = ptr;
		m_nameToSubsys[strName] = ptr;
	}

	std::vector<Subsystem*>& SubsystemMgr::GetSubsystems()
	{
		return m_subsystems;
	}

	Subsystem* SubsystemMgr::GetSubsystem(const std::string &strName)
	{
		for(
				std::vector<Subsystem*>::iterator iter = m_subsystems.begin();
				iter != m_subsystems.end();
				iter++)
		{
			if((*iter)->GetName().compare(strName) == 0) return *iter;
			std::cout<<(*iter)->GetName()<<std::endl;
		}
		return NULL;
	}

	Subsystem::Subsystem(const std::string &strName) : m_strName(strName), m_usPeriod(1)
	{
		SubsystemMgr::GetSingleton()->RegisterSubsystem(strName, this);
	}

	Subsystem::~Subsystem()
	{
	}
};
