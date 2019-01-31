#pragma once

#pragma region Controller info

#define CONTROLLER_ESC		0
#define CONTROLLER_SERVO	1

#pragma endregion

#pragma region Id offsets

#define ESC_ID_OFFSET		0
#define SERVO_ID_OFFSET		8

#pragma endregion

/// <summary>
/// Current node firmware
/// </summary>
#define CONTROLLER			CONTROLLER_SERVO	// Manually changeable

#pragma region Including of specified controller

#if CONTROLLER == CONTROLLER_ESC
	#include "ESCController.hpp"
	#include "ESCDevices.hpp"
#elif CONTROLLER == CONTROLLER_SERVO
	#include "ServoController.hpp"
	#include "ServoDevices.hpp"
#endif

#pragma endregion

namespace Controllers
{
	/// <summary>
	/// Current device id
	/// </summary>
	const uint32_t deviceId	= (unsigned)
#if CONTROLLER == CONTROLLER_ESC
		ESCDevices::
#elif CONTROLLER == CONTROLLER_SERVO
		ServoDevices::
#endif
			Throttle;             	// Manually changeable

	/// <summary>
	/// Real node id in CAN bus
	/// </summary>
	const uint32_t nodeId	= (deviceId + (CONTROLLER == CONTROLLER_ESC ? ESC_ID_OFFSET : SERVO_ID_OFFSET));
}