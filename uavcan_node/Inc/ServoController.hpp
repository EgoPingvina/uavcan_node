#pragma once

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <config/config_storage.hpp>

#include "CanNode.hpp"

namespace Controllers
{	
	#define NODE_NAME "io.px4.actuator"

	class ServoController : public CanNode
	{
	public:
		ServoController();
	
		void Initialize() override;

		void ConfigureNode() override;
	
		bool GetValue(int32_t* value) override;

	private:	
		typedef uavcan::MethodBinder<ServoController*,
			void(ServoController::*)(const uavcan::TimerEvent&)>
			    StatusCallbackBinder;
	
		typedef uavcan::MethodBinder<ServoController*,
			void(ServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>&)>
			    ArrayCommandCallbackBinder;

		static constexpr unsigned configStorageSize		= 1024;

		void* const configStorageAddress;
		
		/// <summary>
		/// Number of servo devices on this can node
		/// </summary>
		const unsigned deviceCount;
	
		float position									= 0.0F; 
	
		float force										= 0.0F;
	
		float speed										= 0.0F; 
	
		int32_t *value;
	
		uavcan::Publisher<uavcan::equipment::actuator::Status>* statusPublisher;
	
		os::config::Param<unsigned> paramNodeId;
	
		os::config::Param<unsigned> paramActuatorId;
	
		os::config::Param<float> paramPosition;
	
		os::config::Param<float> paramForce;
	
		os::config::Param<float> paramSpeed;
	
		os::config::Param<unsigned> paramPowerRatingPct;

		/// <summary>
		/// Uses by timer for sending Status with some period
		/// </summary>
		void StatusCallback(const uavcan::TimerEvent& event);
	
		/// <summary>
		/// Callback of new ArrayCommand package
		/// </summary>
		void ArrayCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>& msg);
	};
}