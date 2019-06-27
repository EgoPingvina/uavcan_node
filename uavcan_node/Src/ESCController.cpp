#include "ESCController.hpp"

using namespace Controllers;

ESCController::ESCController()
	: configStorageAddress(reinterpret_cast<void*>(0x08000000 + (512 * 1024) - 1024))
	, paramESCIndex("esc_index", deviceId, 0, 15)
#ifdef PARAM_SERVER	
	, paramManager(&this->selfIndex, &this->selfDirection)
#endif
{ }

void ESCController::Initialize()
{
	this->ConfigureNode();

	int32_t isOk = 0;

#pragma region Subscription initialize
	
	static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(this->GetNode());
	isOk = sub_raw_command.start(
		RawCommandCallbackBinder(
			this,
			&ESCController::RawCommandCallback));	
	if (isOk < 0)
		this->ErrorHandler(__LINE__);

#pragma endregion

#pragma region Publications initialize

	static uavcan::Timer statusSender(this->GetNode());
	
	this->statusPublisher = new uavcan::Publisher<uavcan::equipment::esc::Status>(this->GetNode());
	isOk = this->statusPublisher->init();
	if (isOk != 0)
		this->ErrorHandler(__LINE__);
	
	statusSender.setCallback(
		StatusCallbackBinder(
			this,
			&ESCController::StatusCallback));
	
	statusSender.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));  	// 10Hz

#pragma endregion
}

void ESCController::ConfigureNode()
{
	auto& mynode = this->GetNode();

	if (!mynode.setNodeID(nodeId))
		this->ErrorHandler(__LINE__);

	mynode.setName(NODE_NAME);

	uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
	sw_version.major		= 2;
	sw_version.minor		= 0;
	sw_version.vcs_commit	= 0xb1354f4; // git rev-parse --short HEAD
	sw_version.image_crc	= 0x0102030405060708; // image CRC-64-WE see https://uavcan.org/Specification/7._List_of_standard_data_types/ 
	sw_version.optional_field_flags |= sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	mynode.setSoftwareVersion(sw_version);

	uavcan::protocol::HardwareVersion hw_version;  // Standard type uavcan.protocol.HardwareVersion
	hw_version.major		= 1;
	hw_version.minor		= 0;
	const auto uid			= this->ReadUID();
	std::copy(std::begin(uid), std::end(uid), std::begin(hw_version.unique_id));
	mynode.setHardwareVersion(hw_version);

	/*
	* Start the node.
	* All returnable error codes are listed in the header file uavcan/error.hpp.
	*/
	if (mynode.start() < 0)
		this->ErrorHandler(__LINE__);

	mynode.setRestartRequestHandler(&restart_request_handler);
	
#ifdef PARAM_SERVER		
	// initialization
	if (get_param_server().start(&this->paramManager) < 0)
		this->ErrorHandler(__LINE__);
#endif 		

#ifdef ENUMERATION		
	enumeration_handler_.construct<uavcan::INode&>(mynode);
	if (enumeration_handler_->start() < 0)
		this->ErrorHandler(__LINE__);
#endif 

	mynode.setModeOperational();

	// Config
	static os::stm32::ConfigStorageBackend config_storage_backend(configStorageAddress, configStorageSize);
	if (os::config::init(&config_storage_backend) < 0)
		this->ErrorHandler(__LINE__);

	this->selfIndex = this->paramESCIndex.get();
}
	
bool ESCController::GetValue(int32_t* value)
{
	if (!this->IsValueUpdate())
		return false;
		
	this->isValueUpdate = false; 
	*value = this->value; 
	return true;
}

void ESCController::Output()
{
	int32_t value = 0;
	if (this->GetValue(&value))
		TIM3->CCR2 = TIM3->CCR1 =
			value < 1
				? 900
				: NumericConvertions::RangeTransform<1, 8191, 1100, 1900>(value);
}

void ESCController::StatusCallback(const uavcan::TimerEvent& event)
{
	uavcan::equipment::esc::Status message;

	// ToDo send real data
	message.esc_index			= this->selfIndex;
	message.rpm					= 0;
	message.voltage				= 3.3F;
	message.current				= 0.001F;
	message.temperature			= 24.0F;
	message.power_rating_pct	= static_cast<unsigned>(.5F * 100 + 0.5F);
	message.error_count			= 0;

	if (this->statusPublisher->broadcast(message) < 0)
		this->ErrorHandler(__LINE__);
}

void ESCController::RawCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg)
{
	if (msg.cmd.size() <= this->selfIndex)
	{
		if (!this->IsValueUpdate())
			this->isValueUpdate = true;

		this->value = 0;
		return;
	}

	this->value = msg.cmd[this->selfIndex];
	this->isValueUpdate = true;
}