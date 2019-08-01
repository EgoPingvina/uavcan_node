#pragma once

#include "ServoController.hpp"

#include <uavcan/equipment/big_one/Preflight_state.hpp>

namespace Controllers
{
	class TailWithPreflight : public ServoController
	{
	public:
		TailWithPreflight();

		void Initialize() override;

		inline void Output() override;

	private:
		typedef uavcan::MethodBinder<TailWithPreflight*,
			void(TailWithPreflight::*)(const uavcan::TimerEvent&)>
			PreflightStateCallbackBinder;

		GPIO_TypeDef *port;
		const uint16_t pin;

		/// <summary>
		/// Time in ms for button click accepting
		/// </summary>
		const int32_t pressTime;


		uavcan::Publisher<uavcan::equipment::big_one::Preflight_state>* preflightStatePublisher;

		/// <summary>
		/// Preflight state (safity switch)
		/// </summary>
		bool isPreflightOn = false;
		
		/// <summary>
		/// Previous state of preflight button
		/// </summary>
		bool previousState = false;
		
		/// <summary>
		/// Last tick  of preflight button state change
		/// </summary>
		uint32_t lastTick;

		/// <summary>
		/// Uses by timer for sending Status with some period
		/// </summary>
		void PreflightStateCallback(const uavcan::TimerEvent& event);
	};
}