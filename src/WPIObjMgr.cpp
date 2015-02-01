
#include "WPIObjMgr.h"
#include "Config.h"
#include <unordered_map>


namespace Spyder
{
	Joystick* GetJoystick(uint32_t port)
	{
		static Joystick *s_joysticks[4] = {0, 0, 0, 0};
		if(!s_joysticks[port])
		{
			s_joysticks[port] = new Joystick(port);
		}
		return s_joysticks[port];
	}

	Relay* GetRelay(uint32_t channel)
	{
		static std::unordered_map<uint32_t, Relay*> s_relays;
		if(!s_relays[channel])
		{
			s_relays[channel] = new Relay(channel, Relay::kBothDirections);
		}
		return s_relays[channel];
	}

	Victor* GetVictor(uint32_t channel)
	{
		static std::unordered_map<uint32_t, Victor*> s_victors;
		if(!s_victors[channel])
		{
			s_victors[channel] = new Victor(channel);
		}
		return s_victors[channel];
	}
	Encoder* GetEncoder(uint32_t aChannel, uint32_t bChannel, bool inverse)
	{
		static std::unordered_map<uint32_t, Encoder*> s_encoders;
		if(!s_encoders[aChannel])
		{
			s_encoders[aChannel] = new Encoder(aChannel, bChannel, inverse);
		}
		return s_encoders[aChannel];
	}

	double GetDeadzone()
	{
		static ConfigVar<double> deadzone("controller_deadzone", 0.15);
		return deadzone.GetVal();
	}

	Solenoid* GetSolenoid(uint32_t channel)
	{
		static std::unordered_map<uint32_t, Solenoid*> s_solenoids;
		if(!s_solenoids[channel])
		{
			s_solenoids[channel] = new Solenoid(channel);
		}
		return s_solenoids[channel];
	}
}
