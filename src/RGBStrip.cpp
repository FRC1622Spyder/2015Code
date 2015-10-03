#include "RGBStrip.h"
#include <iostream>
#include <cstdint>

namespace Spyder
{
	RGBStrip::RGBStrip(uint32_t redChannel, uint32_t greenChannel, uint32_t blueChannel)
		: m_pwmR(redChannel), m_pwmG(greenChannel), m_pwmB(blueChannel)
	{
	}

	void RGBStrip::SetColor(uint16_t R, uint16_t G, uint16_t B)
	{
		/*m_pwmR.SetRaw(R<<8);
		m_pwmG.SetRaw(G<<8);*/
		m_pwmB.SetRaw(B);
		m_pwmR.SetRaw(R);
		m_pwmG.SetRaw(G);
	}

	void RGBStrip::SetR(uint16_t R)
	{
		m_pwmR.SetRaw(R);
	}

	void RGBStrip::SetG(uint16_t G)
	{
		m_pwmR.SetRaw(G);
	}

	void RGBStrip::SetB(uint16_t B)
	{
		m_pwmR.SetRaw(B);
	}
}

