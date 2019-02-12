#include "ServoController.hpp"
#include "ServoDevices.hpp"

using namespace Controllers;

ServoController::ServoController()
	: configStorageAddress(reinterpret_cast<void*>(0x08000000 + (512 * 1024) - 1024))
	, deviceCount(NumericConvertions::UpBitsCount(deviceId))
	, paramNodeId("uavcan_node_id", nodeId, 0, 125)
	, paramActuatorId("actuator_id", deviceId, 0, 15)
	, paramPosition("position", 0, 0, 360)
	, paramForce("force", 1, 0, 1)
	, paramSpeed("speed", 0, 0, 255)
	, paramPowerRatingPct("power_rating_pct", 0, 0, 255)
{
	this->value = (int32_t*)malloc(this->deviceCount * sizeof(int32_t));
}

void ServoController::Initialize()
{
	this->ConfigureNode();

	int32_t isOk = 0;
	
#pragma region Subscription initialize
	
	static uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand> arrayCommandSubscriber(this->GetNode());
	isOk = arrayCommandSubscriber.start(
		ArrayCommandCallbackBinder(
			this,
			&ServoController::ArrayCommandCallback));	
	if (isOk < 0)
		this->ErrorHandler(__LINE__);

#pragma endregion

#pragma region Publications initialize

	static uavcan::Timer statusSender(this->GetNode());
	
	this->statusPublisher = new uavcan::Publisher<uavcan::equipment::actuator::Status>(this->GetNode());
	isOk = this->statusPublisher->init();
	if (isOk != 0)
		this->ErrorHandler(__LINE__);
	
	statusSender.setCallback(
		StatusCallbackBinder(
			this,
			&ServoController::StatusCallback));
	
	statusSender.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));   	// 10Hz

#pragma endregion
}

void ServoController::ConfigureNode()
{
	auto& mynode = this->GetNode();

	if (!mynode.setNodeID(nodeId))
		this->ErrorHandler(__LINE__);

	mynode.setName(NODE_NAME);

	uavcan::protocol::SoftwareVersion sw_version;   // Standard type uavcan.protocol.SoftwareVersion
	sw_version.major		= 2;
	sw_version.minor		= 0;
	sw_version.vcs_commit	= 0xb1354f4;  // git rev-parse --short HEAD
	sw_version.image_crc	= 0x0102030405060708;  // image CRC-64-WE see https://uavcan.org/Specification/7._List_of_standard_data_types/ 
	sw_version.optional_field_flags |= sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	mynode.setSoftwareVersion(sw_version);

	uavcan::protocol::HardwareVersion hw_version;   // Standard type uavcan.protocol.HardwareVersion
	hw_version.major		= 1;
	hw_version.minor		= 0;
	const auto uid			= this->ReadUID();
	std::copy(std::begin(uid), std::end(uid), std::begin(hw_version.unique_id));
	mynode.setHardwareVersion(hw_version);
	
	/*
	 * Set for uavcan::equipment::actuator::Command custom ID 1012
	 */
	if (uavcan::GlobalDataTypeRegistry::instance().registerDataType<uavcan::equipment::actuator::Command>(1012) != uavcan::GlobalDataTypeRegistry::RegistrationResultOk)
		this->ErrorHandler(__LINE__);

	/*
	* Start the node.
	* All returnable error codes are listed in the header file uavcan/error.hpp.
	*/
	if (mynode.start() < 0)		
		this->ErrorHandler(__LINE__);

	mynode.setRestartRequestHandler(&restart_request_handler);

	mynode.setModeOperational();

#pragma region Config

	static os::stm32::ConfigStorageBackend config_storage_backend(configStorageAddress, configStorageSize);
	if (os::config::init(&config_storage_backend) < 0)
		this->ErrorHandler(__LINE__);

	this->selfIndex = this->paramActuatorId.get();
	this->position	= this->paramPosition.get();
	this->force		= this->paramForce.get();
	this->speed		= this->paramSpeed.get();
	
#pragma endregion
}
	
bool ServoController::GetValue(int32_t* value)
{
	if (!this->IsValueUpdate())
		return false;
	
	this->isValueUpdate = false; 
	memcpy(value, this->value, this->deviceCount * sizeof(int32_t));
	return true;
}
		
void ServoController::Output()
{	
	static volatile uint32_t* outputChannels[3] = { &TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3 };
	
	int32_t value[this->deviceCount];
	if (this->GetValue(value))
		for (uint32_t i = 0; i < deviceCount; i++)
			*outputChannels[i] = 
				value[i] < 1
					? 1500		// default value for actuators - middle position
					: Controllers::deviceId == (uint32_t)ServoDevices::AileronLeft || Controllers::deviceId == (uint32_t)ServoDevices::AileronRight
						? NumericConvertions::RangeTransform<1125, 1875, 1000, 2000>(value[i])		// ailerons
						: NumericConvertions::RangeTransform<1000, 2000, 1100, 1900>(value[i]);  	// tail(rudder & elevator)
}

void ServoController::StatusCallback(const uavcan::TimerEvent& event)
{
	uavcan::equipment::actuator::Status message;

	// ToDo send real data
	message.actuator_id			= this->selfIndex;
	message.force				= this->force; 
	message.position			= this->position;
	message.speed				= this->speed; 
	message.power_rating_pct	= 0.0F;

	if (this->statusPublisher->broadcast(message) < 0)
		this->ErrorHandler(__LINE__);
}

void ServoController::ArrayCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>& message)
{
	if (message.commands.size() < (unsigned)ServoDevices::Size)
	{
		if (!this->isValueUpdate)
			this->isValueUpdate = true;

		this->value = 0;
		return;
	}

	for (unsigned i = 0, position = 0; i < (unsigned)ServoDevices::Size; i++)
		if (GetBit(deviceId, i) == 1)
			if (message.commands[i].command_type == (uint8_t)Commands::PWM)
				this->value[position++] = message.commands[i].command_value;
	
	this->isValueUpdate = true;
}