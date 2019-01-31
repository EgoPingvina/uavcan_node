#pragma once

#include <type_traits>

enum class ServoDevices : uint32_t
{
	AileronLeft		= 0x01,					// AUX1 : Aileron 1
	AileronRight	= 0x02,					// AUX2 : Aileron 2
	Elevator		= 0x04,					// AUX3 : Elevator
	Rudder			= 0x08,					// AUX4 : Rudder
	Throttle		= 0x10,					// AUX5 : Throttle
	
	Size			= 5,
	
	Tail			= Rudder | Elevator		// this node has 2 actuators: rudder and elevator
};

constexpr inline ServoDevices operator |(ServoDevices left, ServoDevices right)
{
	using T = std::underlying_type_t <ServoDevices>;
	return static_cast<ServoDevices>(static_cast<T>(left) | static_cast<T>(right));
}

constexpr inline ServoDevices& operator |=(ServoDevices& left, ServoDevices right)
{
	left = left | right;
	return left;
}