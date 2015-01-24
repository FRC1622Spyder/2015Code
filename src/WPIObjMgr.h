#pragma once
#include <WPILib.h>
#include <cstdint>

namespace Spyder
{
	Joystick *GetJoystick(uint32_t port);
	Relay *GetRelay(uint32_t channel);
	Victor* GetVictor(uint32_t channel);
	Solenoid* GetSolenoid(uint32_t channel);
	Encoder* GetEncoder(uint32_t aChannel, uint32_t bChannel, bool inverse);
	double GetDeadzone();
};
