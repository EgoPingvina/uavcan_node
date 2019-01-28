#pragma once

#include <array>

#include <uavcan_stm32/uavcan_stm32.hpp>

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/node/generic_subscriber.hpp>
#include <uavcan/node/node.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>

#include <config/config.hpp>
#include <config/config_storage.hpp>

extern __weak const unsigned deviceId;
extern __weak const unsigned nodeId; 	// node-id for servos in range 9-21

namespace
{	
	#define NODE_NAME "io.px4.actuator"
	
	/// <summary>
	/// Memory pool size largely depends on the number of CAN ifaces and on application's logic.
	/// Please read the documentation for the class uavcan::Node to learn more.
	/// </summary>
	static constexpr unsigned nodeMemoryPoolSize = 16384;   // 4KB - 512KB
	
	typedef uavcan::Node<nodeMemoryPoolSize> Node;
	
	typedef std::array<std::uint8_t, 12> UniqueID;
}

class ServoController
{
public:
	ServoController();
	
	int Initialize();

	int ConfigureNode();

	/// <summary>
	/// Spinning for n second.
	/// The method spin() may return earlier if an error occurs(e.g.driver failure).
	/// All error codes are listed in the header uavcan / error.hpp.
	/// </summary>
	int NodeSpin() const;
	
	unsigned SelfIndex() const;
	
	bool IsValueUpdate(void) const;
	
	bool GetRaw(int* raw);

private:	
	typedef uavcan::MethodBinder<ServoController*,
        void(ServoController::*)(const uavcan::TimerEvent&) const>
            StatusCallbackBinder;
	
	typedef uavcan::MethodBinder<ServoController*,
        void(ServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>&)>
            ArrayCommandCallbackBinder;

	static constexpr int rxQueueSize				= 64;

	static constexpr uint32_t bitRate				= 1000000;

	static constexpr unsigned configStorageSize		= 1024;

	void* const configStorageAddress;
	
	float position									= 0.0F; 
	
	float force										= 0.0F;
	
	float speed										= 0.0F; 
	
	/// <summary>
	/// Value from CAN for this ESC
	/// </summary>
	int rawValue;

	unsigned selfIndex; 
	
	bool isValueUpdate = false;
	
	uavcan::Publisher<uavcan::equipment::actuator::Status>* statusPublisher;
	
	os::config::Param<unsigned> paramNodeId;
	
	os::config::Param<unsigned> paramActuatorId;
	
	os::config::Param<float> paramPosition;
	
	os::config::Param<float> paramForce;
	
	os::config::Param<float> paramSpeed;
	
	os::config::Param<unsigned> paramPowerRatingPct;
	
	/// <summary>
	/// Node object will be constructed at the time of the first access.
	/// Note that most library objects are noncopyable(e.g.publishers, subscribers, servers, callers, timers, ...).
	/// Attempt to copy a noncopyable object causes compilation failure.
	/// </summary>
	Node& GetNode() const;

	/// <summary>
	/// Uses by timer for sending Status with some period
	/// </summary>
	void StatusCallback(const uavcan::TimerEvent& event) const;
	
	/// <summary>
	/// Callback of new ArrayCommand package
	/// </summary>
	void ArrayCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>& msg);

	UniqueID ReadUID() const;

	uavcan::ICanDriver& GetCanDriver() const;
	
	/// <summary>
	/// Restart handler
	/// </summary>
	class RestartRequestHandler : public uavcan::IRestartRequestHandler
	{
		bool handleRestartRequest(uavcan::NodeID request_source) override
		{
			return true;
		}
	} restart_request_handler;
};