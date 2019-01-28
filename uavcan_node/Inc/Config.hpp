#pragma once

#pragma region Controller info

#define CONTOLLER_ESC		0
#define CONTOLLER_SERVO		1

#pragma endregion

#pragma region Servos Info

#define AILERON_1			0	//AUX1 : Aileron 1
#define AILERON_2			1	//AUX2 : Aileron 2
#define ELEVATOR			2	//AUX3 : Elevator
#define RUDDER				3	//AUX4 : Rudder
#define THROTTLE			4	//AUX5 : Throttle

#pragma endregion

#pragma region ESC Info

#define LEFT_TOP			2
#define LEFT_BOTTOM			1
#define RIGHT_TOP			0
#define RIGHT_BOTTOM		3

#pragma endregion

#pragma region Id offsets

#define ESC_ID_OFFSET		0
#define SERVO_ID_OFFSET		8

#pragma endregion

/// <summary>
/// Current node firmware
/// </summary>
#define CONTROLLER			CONTOLLER_ESC	// Manually changeable

#pragma region Including of specified controller

#if CONTROLLER == CONTOLLER_ESC
	#include "ESCController.hpp"
#elif CONTROLLER == CONTOLLER_SERVO
	#include "ServoController.hpp"
#endif

#pragma endregion

/// <summary>
/// Current device id
/// </summary>
const unsigned deviceId =	RIGHT_TOP;		// Manually changeable

/// <summary>
/// Real node id in CAN bus
/// </summary>
const unsigned nodeId =		(deviceId + (CONTROLLER == CONTOLLER_ESC ? ESC_ID_OFFSET : SERVO_ID_OFFSET));