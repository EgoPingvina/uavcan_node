#pragma once
	
#include <bitset>
#include <climits>

namespace Controllers
{	
	#define GetBit(number, position) ((number >> position) & 1)
	#define SetBit(number, position, value) (number | value << position)

	extern __weak const unsigned deviceId;
	extern __weak const unsigned nodeId;

	template<typename T>
	constexpr inline unsigned UpBitsCount(T n)
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");
		
		std::bitset<sizeof(T) * CHAR_BIT> b(n);
		return b.count();
	}
	
//	class CanNode
//	{
//		
//	};
	
}