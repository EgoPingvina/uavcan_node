#include "TailWithPreflight.hpp"

using namespace Controllers;

TailWithPreflight::TailWithPreflight()
	: ServoController()
	, pressTime(1500)	// 1.5s
	, port(GPIOB)
	, pin(GPIO_PIN_0)
{ }

void TailWithPreflight::Initialize()
{
	int32_t isOk = 0;

	ServoController::Initialize();

#pragma region Publications initialize

	static uavcan::Timer preflightStateSender(this->GetNode());

	this->preflightStatePublisher = new uavcan::Publisher<uavcan::equipment::big_one::Preflight_state>(this->GetNode());
	isOk = this->preflightStatePublisher->init();
	if (isOk != 0)
		this->ErrorHandler(__LINE__);

	preflightStateSender.setCallback(
		PreflightStateCallbackBinder(
			this,
			&TailWithPreflight::PreflightStateCallback));

	preflightStateSender.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));  	// 10Hz

#pragma endregion
}

void TailWithPreflight::Output()
{
	ServoController::Output();

	// current state of physical button
	bool currentState			= HAL_GPIO_ReadPin(this->port, this->pin) == GPIO_PIN_RESET;	// true when pressed

	// if button's physical and logical states are different
	if(currentState != this->previousState)
	{
		// take tick of this moment
		uint32_t tick			= HAL_GetTick();
		// if the button maintains its physical state for this->pressTime ms - change its logical state
		if (tick - this->lastTick >= this->pressTime)
		{
			if (currentState && !this->previousState)
				this->isPreflightOn = !this->isPreflightOn;
			
			this->previousState	= currentState;
			this->lastTick		= tick;
		}
	}
	else
		// if physical and logical states are equal - update lastTick
		this->lastTick			= HAL_GetTick();
}

void TailWithPreflight::PreflightStateCallback(const uavcan::TimerEvent& event)
{
	uavcan::equipment::big_one::Preflight_state message;

	message.status = this->isPreflightOn;

	if (this->preflightStatePublisher->broadcast(message) < 0)
		this->ErrorHandler(__LINE__);
}