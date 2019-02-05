#pragma once

#pragma region Controller info

#define CONTROLLER_ESC		0
#define CONTROLLER_SERVO	1
#define MARSHAL_ENGINE		2

#pragma endregion

#pragma region Id offsets

#define ESC_ID_OFFSET		1
#define SERVO_ID_OFFSET		9

#pragma endregion

/// <summary>
/// Current node firmware
/// </summary>
#define CONTROLLER			MARSHAL_ENGINE	// Manually changeable

#pragma region Including of specified controller

#if CONTROLLER == CONTROLLER_ESC
	#include "ESCController.hpp"
	#include "ESCDevices.hpp"
#elif CONTROLLER == CONTROLLER_SERVO
	#include "ServoController.hpp"
	#include "ServoDevices.hpp"
#elif CONTROLLER == MARSHAL_ENGINE
	#include "ServoDevices.hpp"
	#include "MarshalEngine.hpp"
#endif

#pragma endregion

namespace Controllers
{
	/// <summary>
	/// Current device id
	/// </summary>
	const uint32_t deviceId	= (uint32_t)
#if CONTROLLER == MARSHAL_ENGINE
		ServoDevices::Throttle;
#else
	#if CONTROLLER == CONTROLLER_ESC
			ESCDevices::
	#elif CONTROLLER == CONTROLLER_SERVO
			ServoDevices::
	#endif
				AileronLeft;             	// Manually changeable
#endif

	/// <summary>
	/// Real node id in CAN bus
	/// </summary>
	const uint32_t nodeId	= (deviceId + (CONTROLLER == CONTROLLER_ESC ? ESC_ID_OFFSET : SERVO_ID_OFFSET));
}

/// <summary>
/// Delay before can node initialization
/// </summary>
const uint32_t startDelayMs = 5000;			// Manually changeable