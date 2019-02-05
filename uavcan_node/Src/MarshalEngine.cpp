#include "MarshalEngine.hpp"

using namespace Controllers;

MarshalEngine::MarshalEngine()
	: defaultThrottle(950)
	, throttleValue(defaultThrottle)
	, ignitionValue(false)
	, wasStopped(false)
{	
	int32_t isOk = 0;
	
	static uavcan::Subscriber<uavcan::equipment::actuator::Command> arrayCommandSubscriber(this->GetNode());
	isOk = arrayCommandSubscriber.start(
		IgnitionCallbackBinder(
			this,
		&MarshalEngine::IgnitionCallback));	
	if (isOk < 0)
		this->ErrorHandler(__LINE__);
}
		
int32_t MarshalEngine::Throttle()
{
	this->GetValue(&this->throttleValue);
	return this->throttleValue;
}
		
bool MarshalEngine::Ignition() const
{
	return !this->wasStopped && this->ignitionValue;
}

void MarshalEngine::OnStep()
{
	HAL_Delay(20);
	
	TIM3->CCR1 =
		this->Throttle() < 1
			? defaultThrottle
			: NumericConvertions::RangeTransform<1000, 2000, 1000, 1780>(this->throttleValue);
	
	HAL_GPIO_WritePin(
		GPIOB,
		GPIO_PIN_0,
		this->ignitionValue ? GPIO_PIN_RESET : GPIO_PIN_SET);	// key logic is inverted
}

void MarshalEngine::IgnitionCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Command>& message)
{
	if (message.command_type == (uint8_t)Commands::Ignition)
		this->ignitionValue = message.command_value;
	
	if (!this->ignitionValue)
		this->wasStopped = true;
	
	this->isValueUpdate = true;
}