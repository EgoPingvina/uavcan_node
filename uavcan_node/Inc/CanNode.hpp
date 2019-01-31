#pragma once

#include "NumericConvertions.hpp"

namespace Controllers
{	
	#define GetBit(number, position) ((number >> position) & 1)
	#define SetBit(number, position, value) (number | value << position)

	extern __weak const unsigned deviceId;
	extern __weak const unsigned nodeId;
	
	class CanNode
	{
	public:
		//void SetErrorHandler
	};	
}