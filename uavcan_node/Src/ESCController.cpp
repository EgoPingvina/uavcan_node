#include "stm32f1xx_hal.h"

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/HardwareVersion.hpp>
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/bxcan.hpp>
#include <uavcan/error.hpp>

#include "ESCController.hpp"

ESCController::ESCController()
	: configStorageAddress(reinterpret_cast<void*>(0x08000000 + (512 * 1024) - 1024))
	, param_esc_index("esc_index", SELF_INDEX_DEFAULT, 0, 15)
#ifdef PARAM_SERVER	
	, paramManager(&this->selfIndex, &this->selfDirection)
#endif
{
	
}

int ESCController::Initialize()
{
	this->ConfigureNode();

	int isOk = 0;

#pragma region Subscription initialize
	
	static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(this->GetNode());
	isOk = sub_raw_command.start(RawCommandCallback);	
	if (isOk != 0)
		return isOk;

#pragma endregion


#pragma region Publications initialize

	static uavcan::Timer statusPublisher(this->GetNode());

	this->pub_status = new uavcan::Publisher<uavcan::equipment::esc::Status>(this->GetNode());
	isOk = this->pub_status->init();
	if (isOk != 0)
		return isOk;

	statusPublisher.setCallback(&StatusCallback);
	statusPublisher.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));	// 10Hz

#pragma endregion

	return isOk;
}

int ESCController::ConfigureNode()
{
	auto& mynode = this->GetNode();

	mynode.setNodeID(NODE_ID);

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
	const auto uid			= read_unique_id();
	std::copy(std::begin(uid), std::end(uid), std::begin(hw_version.unique_id));
	mynode.setHardwareVersion(hw_version);

	/*
	* Start the node.
	* All returnable error codes are listed in the header file uavcan/error.hpp.
	*/
	const int node_start_res = mynode.start();
	if (node_start_res != 0)
		return node_start_res;

	mynode.setRestartRequestHandler(&restart_request_handler);
	
#ifdef PARAM_SERVER		
	// инициализация 
	int param_server_res = get_param_server().start(&this->paramManager);
	if (param_server_res < 0)
		return param_server_res;
#endif 		

#ifdef ENUMERATION		
	enumeration_handler_.construct<uavcan::INode&>(mynode);
	int enumeration_handler_res = enumeration_handler_->start();
	if (enumeration_handler_res < 0)
		return enumeration_handler_res;
#endif 

	mynode.setModeOperational();

	// Config
	static os::stm32::ConfigStorageBackend config_storage_backend(configStorageAddress, configStorageSize);
	const int config_init_res = os::config::init(&config_storage_backend);

	if (config_init_res < 0)
		return -1;

	this->selfIndex = param_esc_index.get();

	return 0;
}

Node& ESCController::GetNode() const
{
	static Node mynode(this->GetCanDriver(), uavcan_stm32::SystemClock::instance());
	return mynode;
}	

int ESCController::NodeSpin() const
{
	return this->GetNode().spin(uavcan::MonotonicDuration::fromMSec(100));
}

bool ESCController::IsRawUpdate(void) const
{		 
	return this->isRawUpdate;
}
	
bool ESCController::GetRaw(int* raw)
{
	if (!this->IsRawUpdate())
		return false;
		
	this->isRawUpdate = false; 
	*raw = this->rawValue; 
	return true;
}

UniqueID ESCController::read_unique_id() const
{
	UniqueID uid;

	// Unique device ID register has address 0x1FFFF7E8
	std::memcpy(uid.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);

	return uid;
}

uavcan::ICanDriver& ESCController::GetCanDriver() const
{
	static uavcan_stm32::CanInitHelper<rxQueueSize> can;

	static bool initialized = false;

	if (!initialized)
	{
		initialized = true;
		if (can.init(bitRate) != 0)
		{
			// ToDo init error uavcan::ErrDriver;
		}
	}

	return can.driver;
}

void ESCController::StatusCallback(const uavcan::TimerEvent& event) const
{
	uavcan::equipment::esc::Status message;

	message.esc_index			= this->selfIndex;
	message.rpm					= 0;
	message.voltage				= 3.3F;
	message.current				= 0.001F;
	message.temperature			= 24.0F;
	message.power_rating_pct	= static_cast<unsigned>(.5F * 100 + 0.5F);
	message.error_count			= 0;

	if (pub_status->broadcast(message) < 0)
	{
		// ToDo error handler
	}
}

void ESCController::RawCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg)
{
	if (msg.cmd.size() <= this->selfIndex)
	{
		if (!this->isRawUpdate)
			this->isRawUpdate = true;

		this->rawValue = 0;
		return;
	}

	this->rawValue = msg.cmd[this->selfIndex];
	this->isRawUpdate = true;
}