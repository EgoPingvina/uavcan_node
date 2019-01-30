#pragma once

enum class ServoDevices : unsigned
{
	AileronLeft		= 0x01,					// AUX1 : Aileron 1
	AileronRight	= 0x02,					// AUX2 : Aileron 2
	Elevator		= 0x04,					// AUX3 : Elevator
	Rudder			= 0x08,					// AUX4 : Rudder
	Throttle		= 0x10,					// AUX5 : Throttle
	
	Size			= 5,
	
	Tail			= Rudder | Elevator		// this node has 2 actuators: rudder and elevator
};

inline ServoDevices operator |(ServoDevices left, ServoDevices right)
{
	return left | right;
	//using T = std::underlying_type_t <SBJFrameDrag>;
	//return static_cast<SBJFrameDrag>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

inline ServoDevices& operator |=(ServoDevices& left, ServoDevices right)
{
	left = left | right;
	return left;
}