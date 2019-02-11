#pragma once

#include "ServoController.hpp"

#include <uavcan/equipment/actuator/Command.hpp>

namespace Controllers
{
	class MarshalEngine : public ServoController
	{
	public:
		MarshalEngine();
		
		void Initialize() override;
		
		int32_t Throttle();
		
		bool Ignition() const;
		
		inline void Output() override;
		
	private:	
		typedef uavcan::MethodBinder<MarshalEngine*,
			void(MarshalEngine::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Command>&)>
			    IgnitionCallbackBinder;
		
		/// <summary>
		/// default value for main engine - idling
		/// </summary>
		const int32_t defaultThrottle;
	
		/// <summary>
		/// Callback of new ignition state value
		/// </summary>
		void IgnitionCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Command>& message);

		void MainLoop();

		int32_t throttleValue;

		bool ignitionValue;
		
		/// <summary>
		/// if the engine has been stopped, it must remain stopped
		/// </summary>
		bool wasStopped;
	};
}