#include "uavcan_node.hpp"
#include "stm32f1xx_hal.h"
#include "main.h"

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/enumeration/Indication.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/HardwareVersion.hpp>
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp> 
#include <uavcan/equipment/indication/LightsCommand.hpp> 
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan/node/node.hpp>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/bxcan.hpp>
#include <uavcan/error.hpp>

#include <unordered_map>
#include <uavcan/protocol/node_info_retriever.hpp>      // For uavcan::NodeInfoRetriever


#define NODE_INFO_LISTENER
//#define NODE_STATUS_MONITOR

namespace uavcan_node
{
	namespace
	{
				
		#define NODE_ID 6
		#define NODE_NAME "io.px4.sapog" //"TestController"

		static constexpr int RxQueueSize = 64;
		static constexpr uint32_t BitRate = 1000000;
		
		uavcan::Publisher<uavcan::equipment::esc::Status>* pub_status;

		static int rpmCnt  = 0;
		static bool rpmUpdate = false, rpmIncrease = true;
		static int error_code = 0;
		
		unsigned self_index = 0;
		int rpm = 0;

		uavcan::ISystemClock& getSystemClock()
		{
			return uavcan_stm32::SystemClock::instance();
		}

		uavcan::ICanDriver& getCanDriver()
		{
			static uavcan_stm32::CanInitHelper<RxQueueSize> can;
			static bool initialized = false;
			if (!initialized)
			{
				initialized = true;
				int res = can.init(BitRate);
				if (res != 0)
				{
					//Handle the error
					error_code = uavcan::ErrDriver;
				}
			}
			return can.driver;
		}


		/**
		* Memory pool size largely depends on the number of CAN ifaces and on application's logic.
		* Please read the documentation for the class uavcan::Node to learn more.
		*/
		constexpr unsigned NodeMemoryPoolSize = 16384; // 4KB - 512 KB

		typedef uavcan::Node<NodeMemoryPoolSize> Node;

		/**
		* Node object will be constructed at the time of the first access.
		* Note that most library objects are noncopyable (e.g. publishers, subscribers, servers, callers, timers, ...).
		* Attempt to copy a noncopyable object causes compilation failure.
		*/
		static Node& getNode()
		{
			static Node mynode(getCanDriver(), getSystemClock());
			return mynode;
		}


#ifdef NODE_STATUS_MONITOR
		/**
 * This class implements a passive node monitor.
 * There's a basic node monitor implementation in the library: uavcan::NodeStatusMonitor
 * Extension through inheritance allows to add more complex logic to it.
 */
		class NodeMonitor : public uavcan::NodeStatusMonitor
		{
			/**
			 * This method is not required to implement.
			 * It is called when a remote node becomes online, changes status, or goes offline.
			 */
			void handleNodeStatusChange(const NodeStatusChangeEvent& event) override
			{
				if (event.was_known)
				{
//					std::cout << "Node " << int(event.node_id.get()) << " has changed status from "
//					          << modeToString(event.old_status) << "/" << healthToString(event.old_status)
//					          << " to "
//					          << modeToString(event.status) << "/" << healthToString(event.status)
//					          << std::endl;
				}
				else
				{
//					std::cout << "Node " << int(event.node_id.get()) << " has just appeared with status "
//					          << modeToString(event.status) << "/" << healthToString(event.status)
//					          << std::endl;
				}
			}

			/**
			 * This method is not required to implement.
			 * It is called for every received message uavcan.protocol.NodeStatus after handleNodeStatusChange(), even
			 * if the status code has not changed.
			 */
			void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg) override
			{
				(void)msg;
				//std::cout << "Remote node status message\n" << msg << std::endl << std::endl;
			}

		public:
			NodeMonitor(uavcan::INode& node)
				: uavcan::NodeStatusMonitor(node)
			{}

			static const char* modeToString(const NodeStatus status)
			{
				switch (status.mode)
				{
				case uavcan::protocol::NodeStatus::MODE_OPERATIONAL:     return "OPERATIONAL";
				case uavcan::protocol::NodeStatus::MODE_INITIALIZATION:  return "INITIALIZATION";
				case uavcan::protocol::NodeStatus::MODE_MAINTENANCE:     return "MAINTENANCE";
				case uavcan::protocol::NodeStatus::MODE_SOFTWARE_UPDATE: return "SOFTWARE_UPDATE";
				case uavcan::protocol::NodeStatus::MODE_OFFLINE:         return "OFFLINE";
				default: return "???";
				}
			}

			static const char* healthToString(const NodeStatus status)
			{
				switch (status.health)
				{
				case uavcan::protocol::NodeStatus::HEALTH_OK:       return "OK";
				case uavcan::protocol::NodeStatus::HEALTH_WARNING:  return "WARNING";
				case uavcan::protocol::NodeStatus::HEALTH_ERROR:    return "ERROR";
				case uavcan::protocol::NodeStatus::HEALTH_CRITICAL: return "CRITICAL";
				default: return "???";
				}
			}
		};
#endif 
		
#ifdef NODE_INFO_LISTENER		

		// listener
		/**
		 * This class will be collecting information from uavcan::NodeInfoRetriever via the interface uavcan::INodeInfoListener.
		 * Please refer to the documentation for uavcan::NodeInfoRetriever to learn more.
		 */
		class NodeInfoCollector final : public uavcan::INodeInfoListener
		{
			struct NodeIDHash
			{
				std::size_t operator()(uavcan::NodeID nid) const { return nid.get(); }
			};

			std::unordered_map<uavcan::NodeID, uavcan::protocol::GetNodeInfo::Response, NodeIDHash> registry_;

			/**
			 * Called when a response to GetNodeInfo request is received. This happens shortly after the node restarts or
			 * becomes online for the first time.
			 * @param node_id   Node ID of the node
			 * @param response  Node info struct
			 */
			void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				const uavcan::protocol::GetNodeInfo::Response& node_info) override
			{
				registry_[node_id] = node_info;
			}

			/**
			 * Called when the retriever decides that the node does not support the GetNodeInfo service.
			 * This method will never be called if the number of attempts is unlimited.
			 */
			void handleNodeInfoUnavailable(uavcan::NodeID node_id) override
			{
				// In this implementation we're using empty struct to indicate that the node info is missing.
				registry_[node_id] = uavcan::protocol::GetNodeInfo::Response();
			}

			/**
			 * This call is routed directly from @ref NodeStatusMonitor.
			 * Default implementation does nothing.
			 * @param event     Node status change event
			 */
			void handleNodeStatusChange(const uavcan::NodeStatusMonitor::NodeStatusChangeEvent& event) override
			{
				if (event.status.mode == uavcan::protocol::NodeStatus::MODE_OFFLINE)
				{
					registry_.erase(event.node_id);
				}
			}

			/**
			 * This call is routed directly from @ref NodeStatusMonitor.
			 * Default implementation does nothing.
			 * @param msg       Node status message
			 */
			void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg) override
			{
				auto x = registry_.find(msg.getSrcNodeID());
				if (x != registry_.end())
				{
					x->second.status = msg;
				}
			}

		public:
			/**
			 * Number if known nodes in the registry.
			 */
			std::uint8_t getNumberOfNodes() const
			{
				return static_cast<std::uint8_t>(registry_.size());
			}

			/**
			 * Returns a pointer to the node info structure for the given node, if such node is known.
			 * If the node is not known, a null pointer will be returned.
			 * Note that the pointer may be invalidated afterwards, so the object MUST be copied if further use is intended.
			 */
			const uavcan::protocol::GetNodeInfo::Response* getNodeInfo(uavcan::NodeID node_id) const
			{
				auto x = registry_.find(node_id);
				if (x != registry_.end())
				{
					return &x->second;
				}
				else
				{
					return nullptr;
				}
			}
		};
		
		NodeInfoCollector collector;
#endif
	}

	
	typedef std::array<std::uint8_t, 12> UniqueID;
	// Unique device ID register has address 0x1FFFF7E8
	UniqueID read_unique_id()
	{
		UniqueID out;
		std::memcpy(out.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);
		return out;
	}

	int configureNode()
	{
		auto& mynode = getNode();

		mynode.setNodeID(NODE_ID);

		mynode.setName(NODE_NAME);

		uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
		sw_version.major = 2;
		sw_version.minor = 0;
		sw_version.vcs_commit = 0xb1354f4; // git rev-parse --short HEAD
		sw_version.image_crc = 0x0102030405060708; // image CRC-64-WE see https://uavcan.org/Specification/7._List_of_standard_data_types/ 
		mynode.setSoftwareVersion(sw_version);

		uavcan::protocol::HardwareVersion hw_version;  // Standard type uavcan.protocol.HardwareVersion
		hw_version.major = 1;
		hw_version.minor = 0;
		const auto uid = read_unique_id();
		std::copy(std::begin(uid), std::end(uid), std::begin(hw_version.unique_id));
		//hw_version.certificate_of_authenticity = 0; // not needed
		mynode.setHardwareVersion(hw_version);
			
		
		/*
		* Start the node.
		* All returnable error codes are listed in the header file uavcan/error.hpp.
		*/
		const int node_start_res = mynode.start();
		if (node_start_res != 0)
		{
			error_code = node_start_res; 
			return node_start_res; 
		}
//		while (true) {
//			const int uavcan_start_res = mynode.start();
//			if (uavcan_start_res >= 0) {
//				break;
//			}
//		}
//		assert(mynode.isStarted());


#ifdef NODE_STATUS_MONITOR
		/*
     * Instantiating the monitor.
     * The object is noncopyable.
     */
		NodeMonitor monitor(mynode);

		/*
		 * Starting the monitor.
		 * Once started, it runs in the background and does not require any attention.
		 */
		const int monitor_start_res = monitor.start();
		if (monitor_start_res < 0)
		{
			//throw std::runtime_error("Failed to start the monitor; error: " + std::to_string(monitor_start_res));
			error_code = monitor_start_res;
		}

		/*
		 * Spinning the node for 2 seconds and then printing the list of nodes in the network.
		 */
		if (mynode.spin(uavcan::MonotonicDuration::fromMSec(2000)) < 0)
		{
			//throw std::runtime_error("Spin failed");
		}

		//std::cout << "Known nodes:" << std::endl;
		for (int i = 1; i <= uavcan::NodeID::Max; i++)
		{
			if (monitor.isNodeKnown(i))
			{
				auto status = monitor.getNodeStatus(i);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//				std::cout << "Node " << i << ": "
//				          << NodeMonitor::modeToString(status) << "/" << NodeMonitor::healthToString(status)
//				          << std::endl;
				/*
				 * It is left as an exercise for the reader to call the following services for each discovered node:
				 *  - uavcan.protocol.GetNodeInfo       - full node information (name, HW/SW version)
				 *  - uavcan.protocol.GetTransportStats - transport layer statistics (num transfers, errors, iface stats)
				 *  - uavcan.protocol.GetDataTypeInfo   - data type check: is supported? how used? is compatible?
				 */
			}
		}

		/*
		 * The monitor provides a method that finds first node with worst health.
		 */
		if (monitor.findNodeWithWorstHealth().isUnicast())
		{
			/*
			 * There's at least one node in the network.
			 */
			auto status = monitor.getNodeStatus(monitor.findNodeWithWorstHealth());
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			//std::cout << "Worst node health: " << NodeMonitor::healthToString(status) << std::endl;
		}
		else
		{
			/*
			 * The network is empty.
			 */
			//std::cout << "No other nodes in the network" << std::endl;
		}
#endif		
#ifdef NODE_INFO_LISTENER		
	/*
     * Initializing the node info retriever object.
     */

		uavcan::NodeInfoRetriever retriever(mynode);

		const int retriever_res = retriever.start();
		if (retriever_res < 0)
		{
			//throw std::runtime_error("Failed to start the retriever; error: " + std::to_string(retriever_res));
			error_code = retriever_res;
		}

		/*
		 * This class is defined above in this file.
		 */
//		NodeInfoCollector collector;

		/*
		 * Registering our collector against the retriever object.
		 * The retriever class may keep the pointers to listeners in the dynamic memory pool,
		 * therefore the operation may fail if the node runs out of memory in the pool.
		 */
		const int add_listener_res = retriever.addListener(&collector);
		if (add_listener_res < 0)
		{
			//throw std::runtime_error("Failed to add listener; error: " + std::to_string(add_listener_res));
			error_code = add_listener_res; 
		}
#endif 

		mynode.setModeOperational();
		
		while (1)
		{
			const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
			if (spin_res != 0)
			{
				error_code = spin_res;
				//return spin_res;
			}
 
#ifdef NODE_INFO_LISTENER
			bool isNodeInfo = false; 
			//for (std::uint8_t i = 1; i <= uavcan::NodeID::Max; i++)
			for(std::uint8_t i = 1 ; i <= 4 ; i++)
			{
				if (auto p = collector.getNodeInfo(i))
				{				
					if (p->name.size() > 0)
					{
						isNodeInfo = true; 	
					}
					//std::cout << "\033[32m---------- " << int(i) << " ----------\033[39m\n" // This line will be colored
					//          << *p << std::endl;				
				}
			}
#endif					
		}

		return 0; 
	}


	int NodeSpin() {

		/////*
		////* Spinning for n second.
		////* The method spin() may return earlier if an error occurs (e.g. driver failure).
		////* All error codes are listed in the header uavcan/error.hpp.
		////*/
		const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(1000));
		if (spin_res != 0)
		{
			error_code = spin_res;
			//return spin_res;
		}
 
#ifdef NODE_INFO_LISTENER
		bool isNodeInfo = false; 
		//for (std::uint8_t i = 1; i <= uavcan::NodeID::Max; i++)
		for (std::uint8_t i = 1; i <= 4; i++)
		{
			if (auto p = collector.getNodeInfo(i))
			{				
				if (p->name.size() > 0)
				{
					isNodeInfo = true; 	
				}
				//std::cout << "\033[32m---------- " << int(i) << " ----------\033[39m\n" // This line will be colored
				//          << *p << std::endl;				
			}
		}
#endif		
		return 0; 
	}
}