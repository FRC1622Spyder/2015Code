#ifndef RGBSTRIP_H_
#define RGBSTRIP_H_

#include <iostream>
#include <string>
#include <cstdint>
#include <WPILib.h>

namespace Spyder
{
	class RGBStrip
	{
	private:
		PWM m_pwmR;
		PWM m_pwmG;
		PWM m_pwmB;
	public:
		RGBStrip(uint32_t redChannel, uint32_t greenChannel, uint32_t blueChannel);
		void SetColor(uint16_t R, uint16_t G, uint16_t B);
		void SetR(uint16_t R);
		void SetG(uint16_t G);
		void SetB(uint16_t B);
	};
}



#endif /* RGBSTRIP_H_ */
