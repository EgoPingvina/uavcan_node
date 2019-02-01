#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>

#include "CallbackT.h"
#include "NumericConvertions.hpp"

#ifndef CANNODE
#define CANNODE

namespace Controllers
{	
	#define GetBit(number, position) ((number >> position) & 1)
	#define SetBit(number, position, value) (number | value << position)

	extern const uint32_t deviceId;
	extern const uint32_t nodeId;
	
	/// <summary>
	/// Memory pool size largely depends on the number of CAN ifaces and on application's logic.
	/// Please read the documentation for the class uavcan::Node to learn more.
	/// </summary>
	static constexpr unsigned nodeMemoryPoolSize = 16384;     // 4KB - 512KB
	
	typedef uavcan::Node<nodeMemoryPoolSize> Node;
	
	typedef std::array<std::uint8_t, 12> UniqueID;
	
	class CanNode
	{
	public:
		virtual void Initialize() = 0;

		virtual void ConfigureNode() = 0;
	
		virtual bool GetValue(int32_t* value) = 0;
		
		/// <summary>
		/// Spinning for n second.
		/// The method spin() may return earlier if an error occurs(e.g.driver failure).
		/// All error codes are listed in the header uavcan / error.hpp.
		/// </summary>
		void NodeSpin();
	
		unsigned SelfIndex() const;
	
		bool IsValueUpdate(void) const;
	
		void SetErrorHandler(const Action<int32_t>& handler)
		{
			this->errorHandler = handler;
		}
		
	protected:	
		unsigned selfIndex;
		
		bool isValueUpdate = false;
		
		UniqueID ReadUID() const;

		uavcan::ICanDriver& GetCanDriver();
		
		/// <summary>
		/// Node object will be constructed at the time of the first access.
		/// Note that most library objects are noncopyable(e.g.publishers, subscribers, servers, callers, timers, ...).
		/// Attempt to copy a noncopyable object causes compilation failure.
		/// </summary>
		Node& GetNode();
		
		void ErrorHandler(int32_t line)
		{
			this->errorHandler(line);
		}
	
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
		
	private :

		static constexpr int32_t rxQueueSize			= 64;

		static constexpr uint32_t bitRate				= 1000000;
		
		Action<int32_t> errorHandler;
	};	
}

#endif // !CANNODE