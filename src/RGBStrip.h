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
		void SetColor(uint8_t R, uint8_t G, uint8_t B);
		void SetR(uint8_t R);
		void SetG(uint8_t G);
		void SetB(uint8_t B);
	};
}



#endif /* RGBSTRIP_H_ */
