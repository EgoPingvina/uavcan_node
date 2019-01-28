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

#include "ServoController.hpp"

ServoController::ServoController()
	: configStorageAddress(reinterpret_cast<void*>(0x08000000 + (512 * 1024) - 1024))
	, paramNodeId("uavcan_node_id", nodeId, 0, 125)
	, paramActuatorId("actuator_id", deviceId, 0, 15)
	, paramPosition("position", 0, 0, 360)
	, paramForce("force", 1, 0, 1)
	, paramSpeed("speed", 0, 0, 255)
	, paramPowerRatingPct("power_rating_pct", 0, 0, 255)
{}

int ServoController::Initialize()
{
	int isOk = 0;

	if ((isOk = this->ConfigureNode()) != 0)
		return isOk;

#pragma region Subscription initialize
	
	static uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand> rawSubscriber(this->GetNode());
	isOk = rawSubscriber.start(
		ArrayCommandCallbackBinder(
			this,
			&ServoController::ArrayCommandCallback));	
	if (isOk != 0)
		return isOk;

#pragma endregion


#pragma region Publications initialize

	static uavcan::Timer statusSender(this->GetNode());
	
	this->statusPublisher = new uavcan::Publisher<uavcan::equipment::actuator::Status>(this->GetNode());
	isOk = this->statusPublisher->init();
	if (isOk != 0)
		return isOk;
	
	statusSender.setCallback(
		StatusCallbackBinder(
			this,
		&ServoController::StatusCallback));
	statusSender.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));   	// 10Hz

#pragma endregion

	return isOk;
}

int ServoController::ConfigureNode()
{
	auto& mynode = this->GetNode();

	mynode.setNodeID(nodeId);

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
	* Start the node.
	* All returnable error codes are listed in the header file uavcan/error.hpp.
	*/
	const int node_start_res = mynode.start();
	if (node_start_res != 0)
		return node_start_res;

	mynode.setRestartRequestHandler(&restart_request_handler);

	mynode.setModeOperational();

#pragma region Config

	static os::stm32::ConfigStorageBackend config_storage_backend(configStorageAddress, configStorageSize);
	const int config_init_res = os::config::init(&config_storage_backend);

	if (config_init_res < 0)
		return -1;

	this->selfIndex = this->paramActuatorId.get();
	this->position	= this->paramPosition.get();
	this->force		= this->paramForce.get();
	this->speed		= this->paramSpeed.get();
	
#pragma endregion

	return 0;
}

Node& ServoController::GetNode() const
{
	static Node mynode(this->GetCanDriver(), uavcan_stm32::SystemClock::instance());
	return mynode;
}	

int ServoController::NodeSpin() const
{
	return this->GetNode().spin(uavcan::MonotonicDuration::fromMSec(100));
}

bool ServoController::IsValueUpdate(void) const
{		 
	return this->isValueUpdate;
}
	
bool ServoController::GetRaw(int* raw)
{
	if (!this->IsValueUpdate())
		return false;
		
	this->isValueUpdate = false; 
	*raw = this->rawValue; 
	return true;
}

UniqueID ServoController::ReadUID() const
{
	UniqueID uid;

	// Unique device ID register has address 0x1FFFF7E8
	std::memcpy(uid.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);

	return uid;
}

uavcan::ICanDriver& ServoController::GetCanDriver() const
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

void ServoController::StatusCallback(const uavcan::TimerEvent& event) const
{
	uavcan::equipment::actuator::Status message;

	// ToDo send real data
	message.actuator_id			= this->selfIndex;
	message.force				= this->force; 
	message.position			= this->position;
	message.speed				= this->speed; 
	message.power_rating_pct	= 0.0F;

	this->statusPublisher->broadcast(message);
}

void ServoController::ArrayCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>& msg)
{
	if (msg.commands.size() <= this->selfIndex)
	{
		if (!this->isValueUpdate)
			this->isValueUpdate = true;

		this->rawValue = 0;
		return;
	}

	this->rawValue = msg.commands[this->selfIndex].command_value;
	this->isValueUpdate = true;
}