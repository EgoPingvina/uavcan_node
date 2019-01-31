#pragma once

enum class ESCDevices : uint32_t
{
	RightTop	= 0,
	LeftTop		= 2,
	RightBottom = 3,
	LeftBottom	= 1
};

inline ESCDevices operator |(ESCDevices left, ESCDevices right)
{
	return left | right;
	//using T = std::underlying_type_t <SBJFrameDrag>;
	//return static_cast<SBJFrameDrag>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

inline ESCDevices& operator |=(ESCDevices& left, ESCDevices right)
{
	left = left | right;
	return left;
}