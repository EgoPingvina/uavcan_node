#include "uavcan_actuator.hpp"
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
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp> 
#include <uavcan/equipment/indication/LightsCommand.hpp> 
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan/node/node.hpp>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/bxcan.hpp>
#include <uavcan/error.hpp>
//#include <uavcan/protocol/restart_request_server.hpp>

#include <unordered_map>
#include <uavcan/protocol/node_info_retriever.hpp>      // For uavcan::NodeInfoRetriever

#include  <config/config.hpp>
#include <config/config_storage.hpp>

//#define NODE_INFO_LISTENER
//#define NODE_STATUS_MONITOR
//#define ENUMERATION
//#define PARAM_SERVER
//#define ENUM_CLIENT
//#define PARAM_CLIENT

#define  AILERON_1	0 //AUX1 : Aileron 1
#define  AILERON_2	1 //AUX2 : Aileron 2
#define  ELEVATOR	2 //AUX3 : Elevator
#define  RUDDER		3 //AUX4 : Rudder
#define  THROTTLE	4 //AUX5 : Throttle

#define SERVO_ID_OFFSET	9		

namespace uavcan_node
{
	namespace
	{
		/*
		* Restart handler
		*/
		class RestartRequestHandler : public uavcan::IRestartRequestHandler
		{
			bool handleRestartRequest(uavcan::NodeID request_source) override
			{
				//os::lowsyslog("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
				//os::requestReboot();
				return true;
			}
		} restart_request_handler;

#define SELF_INDEX_DEFAULT ELEVATOR 		
#define NODE_ID  (SELF_INDEX_DEFAULT + SERVO_ID_OFFSET)// node-id for servos in range 9-21
#define NODE_NAME "io.px4.actuator" 


os::config::Param<unsigned> param_node_id("uavcan_node_id", NODE_ID, 0, 125);
os::config::Param<unsigned> param_actuator_id("actuator_id", SELF_INDEX_DEFAULT, 0, 15);
os::config::Param<float> param_position("position", 0, 0, 360);
os::config::Param<float> param_force("force", 1, 0, 1);
os::config::Param<float> param_speed("speed", 0, 0, 255);
os::config::Param<unsigned> param_power_rating_pct("power_rating_pct", 0, 0, 255);
		
static int error_code = 0;
static bool actuatorsUpdate = false;
static float raw_value; 
			
#define SIM_M_ROTATION_START (NODE_ID * 40)
#define SIM_M_ROTATION_STOP (SIM_M_ROTATION_START + 20)
		unsigned self_index; 
		float position = 0.0F; 
		float force = 0.0F;
		float speed = 0.0F; 
		
		static constexpr int RxQueueSize = 64;
		static constexpr uint32_t BitRate = 1000000;
		
		uavcan::Publisher<uavcan::equipment::actuator::Status>* pub_status;
		
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
		constexpr unsigned NodeMemoryPoolSize = 16384;  // 4KB - 512 KB

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
#ifdef PARAM_CLIENT
		uavcan::ServiceClient<uavcan::protocol::param::GetSet> _enumeration_getset_client(getNode());

		void cb_enumeration_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
		{
			if (!result.isSuccessful()) {
				//warnx("UAVCAN ESC enumeration: save request for node %hhu timed out.", result.getCallID().server_node_id.get());
				error_code = -1;
			}
			else {
				//				warnx("UAVCAN ESC enumeration: save request for node %hhu completed OK.", result.getCallID().server_node_id.get());
				//
				//				uavcan::protocol::param::GetSet::Response resp = result.getResponse();
				//				uint8_t esc_index = (uint8_t)resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>();
				//				esc_index = math::min((uint8_t)(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize - 1), esc_index);
				//				_esc_enumeration_index = math::max(_esc_enumeration_index, (uint8_t)(esc_index + 1));
				//
				//				_esc_enumeration_ids[esc_index] = result.getCallID().server_node_id.get();
				//
				//				uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
				//				opcode_req.opcode = opcode_req.OPCODE_SAVE;
				//				int call_res = _enumeration_save_client.call(result.getCallID().server_node_id, opcode_req);
				//
				//				if (call_res < 0) {
				//					warnx("UAVCAN ESC enumeration: couldn't send ExecuteOpcode: %d", call_res);
				//
				//				}
				//				else {
				//					warnx("UAVCAN ESC enumeration: sent ExecuteOpcode to node %hhu (index %hhu)",
				//						_esc_enumeration_ids[esc_index],
				//						esc_index);
				//				}
			}
		}
#endif 
#ifdef ENUM_CLIENT		
		uavcan::ServiceClient<uavcan::protocol::enumeration::Begin> _enumeration_client(getNode());
		
		void cb_enumeration_begin(const uavcan::ServiceCallResult<uavcan::protocol::enumeration::Begin> &result)
		{
			//uint8_t next_id = get_next_active_node_id(result.getCallID().server_node_id.get());

			if(!result.isSuccessful()) {
				//warnx("UAVCAN ESC enumeration: begin request for node %hhu timed out.", result.getCallID().server_node_id.get());
				error_code = 0;
			}
			else if(result.getResponse().error) {
				//				warnx("UAVCAN ESC enumeration: begin request for node %hhu rejected: %hhu",
				//					result.getCallID().server_node_id.get(),
				//					result.getResponse().error);
									error_code = result.getResponse().error;

			}
			//			else {
			//				_esc_count++;
			//				warnx("UAVCAN ESC enumeration: begin request for node %hhu completed OK.", result.getCallID().server_node_id.get());
			//			}
			//			if (next_id < 128) {
			//				// Still other active nodes to send the request to
			//				uavcan::protocol::enumeration::Begin::Request req;
			//				// TODO: Incorrect implementation; the parameter name field should be left empty.
			//				//       Leaving it as-is to avoid breaking compatibility with non-compliant nodes.
			//				req.parameter_name = "esc_index";
			//				req.timeout_sec = _esc_enumeration_active ? 65535 : 0;
			//
			//				int call_res = _enumeration_client.call(next_id, req);
			//
			//				if (call_res < 0) {
			//					warnx("UAVCAN ESC enumeration: couldn't send Begin request: %d", call_res);
			//
			//				}
			//				else {
			//					warnx("UAVCAN ESC enumeration: sent Begin request");
			//				}
			//
			//			}
			//			else {
			//				warnx("UAVCAN ESC enumeration: begun enumeration on all nodes.");
			//			}
		}
#endif 
		void cb_10Hz(const uavcan::TimerEvent& event)
		{				
#ifdef ENUM_CLIENT			
			// Still other active nodes to send the request to
			uavcan::protocol::enumeration::Begin::Request req;
			// TODO: Incorrect implementation; the parameter name field should be left empty.
			//       Leaving it as-is to avoid breaking compatibility with non-compliant nodes.
			req.parameter_name = "esc_index";
			//req.timeout_sec = _esc_enumeration_active ? 65535 : 0;
			req.timeout_sec = 65535;
			int call_res = _enumeration_client.call(0x06, req);

			if (call_res < 0) {
				//warnx("UAVCAN ESC enumeration: couldn't send Begin request: %d", call_res);
				error_code = call_res;
			} 
#endif
#ifdef PARAM_CLIENT
			uavcan::protocol::param::GetSet::Request req;
			req.name =	"esc_index";  //msg.parameter_name;     // 'esc_index' or something alike, the name is not standardized
			req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = 1;

			int call_res = _enumeration_getset_client.call(0x06, req);

			if (call_res < 0) {
				//warnx("UAVCAN ESC enumeration: couldn't send GetSet: %d", call_res);

			}
			else {
				//warnx("UAVCAN ESC enumeration: sent GetSet to node %hhu (index %d)", _esc_enumeration_ids[i], i);
			}
#endif
			uavcan::equipment::actuator::Status msg;

			msg.actuator_id = self_index;
			msg.force = force; 
			msg.position = position;
			msg.speed = speed; 
			msg.power_rating_pct = 0.0F; //static_cast<unsigned>(.5F * 100 + 0.5F);

			int res = pub_status->broadcast(msg);
			if (res < 0) {
				error_code = res;
			}	
		}
		
		//uavcan::equipment::actuator::Command cmd; 
		
		void cb_array_command(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand>& msg)
		{
			if (msg.commands.size() <= self_index)
			{
				if (!actuatorsUpdate)
					actuatorsUpdate = true;					
					raw_value = 0; 
					return;
			} 
			//msg.commands[self_index].actuator_id
			//msg.commands[self_index].command_type
			raw_value = msg.commands[self_index].command_value; 
			actuatorsUpdate = true;	
		}
		
		void cb_command(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Command>& msg)
		{
		}
		
		static constexpr float BeepFrequencyGenericIndication       = 1000.0F;
		static constexpr float BeepFrequencySuccess                 = 2000.0F;
		static constexpr float BeepFrequencyError                   = 100.0F;
		void cb_beep_command(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::BeepCommand>& msg) {
			
			if (msg.frequency == BeepFrequencyError)
			{
				
			}
			else if (msg.frequency == BeepFrequencySuccess)
			{
							
			}
			else if (msg.frequency == BeepFrequencyGenericIndication)
			{
				
			}			
			//motor_beep(static_cast<int>(msg.frequency), static_cast<int>(msg.duration * 1000));
		}


		void cb_light_command(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::LightsCommand>& msg) {
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		}

		void cb_dynId_command(const uavcan::ReceivedDataStructure<uavcan::protocol::dynamic_node_id::Allocation>& msg) {
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		}
		

		typedef uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate,
			void(*)(const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&,
			uavcan::protocol::file::BeginFirmwareUpdate::Response&)> BeginFirmwareUpdateServer;

		BeginFirmwareUpdateServer& get_begin_firmware_update_server()
		{
			static BeginFirmwareUpdateServer srv(getNode());
			return srv;
		}

		void handle_begin_firmware_update_request(
			const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request,
			uavcan::protocol::file::BeginFirmwareUpdate::Response& response)
		{
			static bool in_progress = false;

			/*::os::lowsyslog("UAVCAN: BeginFirmwareUpdate request from %d\n", int(request.getSrcNodeID().get()));

			if (in_progress) {
				response.error = response.ERROR_IN_PROGRESS;
			}
			else {
				in_progress = true;
				pass_parameters_to_bootloader(active_can_bus_bit_rate, get_node().getNodeID());
				os::requestReboot();
			}*/
		}
	
	
		
#ifdef PARAM_SERVER
		uavcan::ParamServer& get_param_server()
		{
			static uavcan::ParamServer server(getNode());
			return server;
		}
		
		//		typedef enum
		//		{
		//			CONFIG_TYPE_FLOAT,
		//			CONFIG_TYPE_INT,
		//			CONFIG_TYPE_BOOL
		//		} ConfigDataType;
		//
		//		typedef struct
		//		{
		//			const char* name;
		//			float default_;
		//			float min;
		//			float max;
		//			ConfigDataType type;
		//		} ConfigParam;
		//	
		//		const char* configNameByIndex(int index)
		//		{
		//			return nullptr;
		//		}
		//		
		//		int configSet(const char* name, float value) {
		//			return 0; 
		//		}
		//
		//		int configGetDescr(const char* name, ConfigParam* out) {
		//			return 0; 
		//		}
		//
		//		float configGet(const char* name) {
		//			return 0.0; 
		//		}
		//
		//		int configSave(void)
		//		{
		//			return 0;
		//		}
		//
		//		int configErase(void) {
		//			return 0; 
		//		}
		/*
		 * Param access server
		 */
				class ParamManager : public uavcan::IParamManager
		{
			void convert(float native_value, ConfigDataType native_type, uavcan::protocol::param::Value& out_value) const
			{
				if (native_type == CONFIG_TYPE_BOOL) {
					out_value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = bool(native_value != 0);
				}
				else if (native_type == CONFIG_TYPE_INT) {
					out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = native_value;
				}
				else if (native_type == CONFIG_TYPE_FLOAT) {
					out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = native_value;
				}
				else {
					; // :(
				}
			}

			void convertNumeric(float native_value,
				ConfigDataType native_type,
				uavcan::protocol::param::NumericValue& out_value) const
			{
				if (native_type == CONFIG_TYPE_INT) {
					out_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = native_value;
				}
				else if (native_type == CONFIG_TYPE_FLOAT) {
					out_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = native_value;
				}
				else {
					; // :(
				}
			}

			void getParamNameByIndex(Index index, Name& out_name) const override
			{
				const char* name = configNameByIndex(index);
				if (name != nullptr) {
					out_name = name;
				}
			}
			float native_value_last = 0.0; 
			void assignParamValue(const Name& name, const Value& value) override
			{
				Value v = value;
				float native_value = 0.0f;

				if (v.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
					native_value = v.to<uavcan::protocol::param::Value::Tag::boolean_value>() ? 1.0f : 0.0f;
				}
				else if (v.is(uavcan::protocol::param::Value::Tag::integer_value)) {
					native_value = v.to<uavcan::protocol::param::Value::Tag::integer_value>();
					//					native_value_last = native_value; 
					//					const char dir[] = { "ctl_dir" }; 
										const char ind[] = { "esc_index" };
					if (strcmp(name.c_str(), ind) == 0)
					{
						self_index	= value.integer_value;
					}
					//					else if (strcmp(name.c_str(), dir) == 0)
					//					{						
					//						self_dir = value.integer_value;
					//				    }
										HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);	
				}
				else if (value.is(uavcan::protocol::param::Value::Tag::real_value)) {
					native_value = v.to<uavcan::protocol::param::Value::Tag::real_value>();
				}

				(void)configSet(name.c_str(), native_value);
			}

			void readParamValue(const Name& name, Value& out_value) const override
			{
				ConfigParam descr;
				const int res = configGetDescr(name.c_str(), &descr);
				if (res >= 0) {
					convert(configGet(name.c_str()), descr.type, out_value);
					const char dir[] = { "ctl_dir" }; 
					//					const char ind[] = { "esc_index" };
										if(strcmp(name.c_str(), dir) == 0)
					{						
						convert(self_dir, CONFIG_TYPE_INT, out_value); 	
					}
					//					else if (strcmp(name.c_str(), ind) == 0)
					//					{
					//						convert(self_index, CONFIG_TYPE_INT, out_value); 	
					//					}
					//					else 
					//					{
					//						convert(native_value_last, CONFIG_TYPE_INT, out_value); 	
					//					}
				}
			}

			void readParamDefaultMaxMin(const Name& name,
				Value& out_default,
				NumericValue& out_max,
				NumericValue& out_min) const override
			{
				ConfigParam descr;
				const int res = configGetDescr(name.c_str(), &descr);
				if (res >= 0) {
					convert(descr.default_, descr.type, out_default);
					convertNumeric(descr.max, descr.type, out_max);
					convertNumeric(descr.min, descr.type, out_min);
					//					const char dir[] = { "ctl_dir" }; 
					//					const char ind[] = { "esc_index" };
					//					if (strcmp(name.c_str(), dir) == 0)
					//					{						
					//						convert(0, CONFIG_TYPE_INT, out_default);
					//						convertNumeric(1, CONFIG_TYPE_INT, out_max);
					//						convertNumeric(0, CONFIG_TYPE_INT, out_min);
					//					}
					//					else if (strcmp(name.c_str(), ind) == 0)
					//					{
					//						convert(0, CONFIG_TYPE_INT, out_default);
					//						convertNumeric(8, CONFIG_TYPE_INT, out_max);
					//						convertNumeric(0, CONFIG_TYPE_INT, out_min);						
					//					}
				}
			}

			int saveAllParams() override
			{
				// We can't perform flash IO when the motor controller is active
				if(motor_is_idle()) {
					return configSave();
				} else {
					return -1;
				}
			}

			int eraseAllParams() override
			{
				// We can't perform flash IO when the motor controller is active
				if(motor_is_idle()) {
					return configErase();
				} else {
					return -1;
				}
			}
		} param_manager;

#endif 		
#ifdef ENUMERATION		
		/*
		* Enumeration handler
		*/
		class EnumerationHandler : public uavcan::TimerBase
		{
			static constexpr int CONFIRMATION_CHECK_INTERVAL_MSEC = 50;
			
			typedef uavcan::MethodBinder<EnumerationHandler*,
				void(EnumerationHandler::*)
				(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>&,
				uavcan::protocol::enumeration::Begin::Response&)>
CallbackBinder;

			uavcan::ServiceServer<uavcan::protocol::enumeration::Begin, CallbackBinder> srv_;
			uavcan::Publisher<uavcan::protocol::enumeration::Indication> pub_;
			uavcan::MonotonicTime confirmation_deadline_;
			//mutable board::LEDOverlay led_ctl;

			void finish(bool reverse)
			{
				//os::lowsyslog("UAVCAN: Enumeration confirmed: motor reverse: %d\n", (int)reverse);

				uavcan::protocol::enumeration::Indication msg;
				msg.parameter_name = "esc_index";
				pub_.broadcast(msg);

				(void)configSet("ctl_dir", reverse ? 1 : 0);
				
				//motor_stop();  // Shouldn't be running anyway
			}
			
			enum motor_forced_rotation_direction
			{
				MOTOR_FORCED_ROTATION_NONE,
				MOTOR_FORCED_ROTATION_FORWARD,
				MOTOR_FORCED_ROTATION_REVERSE,
			};

			void handleTimerEvent(const uavcan::TimerEvent& event) override
			{
				//led_ctl.blink(board::LEDColor::CYAN);

				if(event.real_time >= confirmation_deadline_) {
					//os::lowsyslog("UAVCAN: Enumeration request expired\n");
					this->stop();
					//led_ctl.unset();
				}
				else {
					const auto rotation = motor_get_forced_rotation_direction();
					//const auto rotation = MOTOR_FORCED_ROTATION_FORWARD;
					if(rotation != MOTOR_FORCED_ROTATION_NONE) {
						const bool reverse = rotation != MOTOR_FORCED_ROTATION_FORWARD;
						finish(reverse);
						//	//led_ctl.unset();
					}	
				}
			}

			void handleBegin(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>& req,
				uavcan::protocol::enumeration::Begin::Response& resp)
			{
				//os::lowsyslog("UAVCAN: Enumeration request from %d, parameter: %s, timeout: %d sec\n",
				//	(int)req.getSrcNodeID().get(), (const char*)req.parameter_name.c_str(), (int)req.timeout_sec);

				/*
				* Checking the parameter name.
				*
				* Empty means that the node is allowed to autodetect what parameter we're enumerating.
				* In the case of Sapog we can enumerate only the ESC index, so it's a no-brainer.
				* If the name is not empty, it must be the name of the ESC index parameter.
				* Currently this name is "esc_index", but it may be changed in the future.
				*
				* However, we'll also need to retain support for "esc_index" even if the parameter gets
				* renamed in the future, as a workaround for incorrect implementation of the enumeration
				* procedure in PX4.
				*
				* Please find the relevant discussion here: https://github.com/PX4/Firmware/pull/2748
				* TL;DR:
				*    There's no such thing as standardized parameter names, except for what's documented here in the
				*    UAVCAN specification:
				*    http://uavcan.org/Specification/6._Application_level_functions/#standard-configuration-parameters
				*/
				if(!req.parameter_name.empty() && (req.parameter_name != "esc_index"))
				{
					//os::lowsyslog("UAVCAN: Enumeration failure - INVALID PARAM NAME\n");
					resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_PARAMETER;
					return;
				}

				//if (!motor_is_idle()) {
				//	//os::lowsyslog("UAVCAN: Enumeration failure - MOTOR CONTROLLER IS NOT IDLE\n");
				//	resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_MODE;
				//	return;
				//}

				confirmation_deadline_ = req.getMonotonicTimestamp() +
					uavcan::MonotonicDuration::fromMSec(req.timeout_sec * 1000);
				this->startPeriodic(uavcan::MonotonicDuration::fromMSec(CONFIRMATION_CHECK_INTERVAL_MSEC));

				resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_OK;
				
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			}

		public:
			EnumerationHandler(uavcan::INode& node)
				: uavcan::TimerBase(node)
				, srv_(node)
				, pub_(node)
			{}

			int start()
			{
				return srv_.start(CallbackBinder(this, &EnumerationHandler::handleBegin));
			}
		};
		
		uavcan::LazyConstructor<EnumerationHandler> enumeration_handler_;		
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
		}
		;
		
		NodeInfoCollector collector;
#endif
	}

	uint32_t last_tick = 0;
	bool raw_increase = true; 
	
	bool getRawUpdate(void) 
	{		 
		return actuatorsUpdate;
	}

	bool getRaw(float* raw) {
		
		if(actuatorsUpdate) {			
			actuatorsUpdate = false; 
			*raw = raw_value; 
			return true; 
		}
		return false;
	}
	
	bool getIntRaw(int* raw)
	{
		if (!actuatorsUpdate)
			return false;
			
		actuatorsUpdate = false; 
		*raw = (int)raw_value; 
		return true; 
	}
	
	unsigned self_esc_index(void)
	{
		return self_index; 
	} 
	
	typedef std::array<std::uint8_t, 12> UniqueID;
	// Unique device ID register has address 0x1FFFF7E8
	UniqueID read_unique_id()
	{
		UniqueID out;
		std::memcpy(out.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);
		return out;
	}
	
	static void* const ConfigStorageAddress = reinterpret_cast<void*>(0x08000000 + (512 * 1024) - 1024);
	constexpr unsigned ConfigStorageSize = 1024;
	
	int configureNode()
	{
		auto& mynode = getNode();

		mynode.setNodeID(NODE_ID);

		mynode.setName(NODE_NAME);

		uavcan::protocol::SoftwareVersion sw_version;   // Standard type uavcan.protocol.SoftwareVersion
		sw_version.major = 2;
		sw_version.minor = 0;
		sw_version.vcs_commit = 0xb1354f4;  // git rev-parse --short HEAD
		sw_version.image_crc = 0x0102030405060708;  // image CRC-64-WE see https://uavcan.org/Specification/7._List_of_standard_data_types/ 
		sw_version.optional_field_flags |= sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
		mynode.setSoftwareVersion(sw_version);

		uavcan::protocol::HardwareVersion hw_version;   // Standard type uavcan.protocol.HardwareVersion
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

		mynode.setRestartRequestHandler(&restart_request_handler);
		//		int res = get_begin_firmware_update_server().start(&handle_begin_firmware_update_request);		
		//		if (res < 0)
		//		{
		//			led_red_toggle(); //board::die(res);
		//		}
#ifdef ENUM_CLIENT
				const int enum_client_init_res = _enumeration_client.init();
		if (enum_client_init_res < 0)
		{			
			error_code = enum_client_init_res;
		}		
		_enumeration_client.setCallback(&cb_enumeration_begin);
#endif	
		
#ifdef PARAM_CLIENT
		const int enum_gs_client_init_res = _enumeration_getset_client.init();
		if (enum_gs_client_init_res < 0)
		{			
			error_code = enum_gs_client_init_res;
		}		
		_enumeration_getset_client.setCallback(&cb_enumeration_getset);
#endif	
		
#ifdef PARAM_SERVER		
		// инициализация 
		int param_server_res = get_param_server().start(&param_manager);
		if (param_server_res < 0) {
			//board::die(res);
			error_code = param_server_res;
		}
#endif 		
		
#ifdef ENUMERATION		
		enumeration_handler_.construct<uavcan::INode&>(mynode);
		int enumeration_handler_res = enumeration_handler_->start();
		if (enumeration_handler_res < 0) {
			//board::die(res);
			//led_red_toggle();
			error_code = enumeration_handler_res;
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
		
		// Config
		static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
		const int config_init_res = os::config::init(&config_storage_backend);
		if (config_init_res < 0)
		{
			error_code = -1;  //die(config_init_res);
		}
		self_index = param_actuator_id.get();
			
		
				return 0; 
	}

	int NodeStartPub() {
			
		int res = 0;

		static uavcan::Timer timer_10hz(getNode());
		

		pub_status = new uavcan::Publisher<uavcan::equipment::actuator::Status>(getNode());
		res = pub_status->init();
		if (res != 0) {
			error_code = res;
			return res;
		}	
		timer_10hz.setCallback(&cb_10Hz);
		timer_10hz.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));
	}

	int NodeStartSub() {

		static uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand> sub_array_command (getNode());
		static uavcan::Subscriber<uavcan::equipment::actuator::Command> sub_command(getNode());
		static uavcan::Subscriber<uavcan::equipment::indication::BeepCommand> sub_beep_command(getNode());
		static uavcan::Subscriber<uavcan::equipment::indication::LightsCommand> sub_light_command(getNode());
		//static uavcan::Subscriber<uavcan::protocol::dynamic_node_id::Allocation> sub_dynId_command(getNode());
		int res = 0;
				
		//		res = sub_esc_status.start(cb_esc_status_command);
		//		if (res != 0) {
		//			return res;
		//		}

		res = sub_array_command.start(cb_array_command);
		if (res != 0) {
			return res;
		}

		res = sub_command.start(cb_command);
		if (res != 0) {
			return res;
		}

//		res = sub_beep_command.start(cb_beep_command);
//		if (res != 0) {
//			return res;
//		}
//
//		res = sub_light_command.start(cb_light_command);
//		if (res != 0) {
//			return res;
//		}

		//		res = sub_dynId_command.start(cb_dynId_command);
		//		if (res != 0) {
		//			return res;
		//		}
			
		//		res = sub_ns_command.start(cb_ns_command);
		//		if (res != 0) {
		//			return res;
		//		}
	}

	int NodeSpin() {

		/////*
		////* Spinning for n second.
		////* The method spin() may return earlier if an error occurs (e.g. driver failure).
		////* All error codes are listed in the header uavcan/error.hpp.
		////*/
		const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
		if (spin_res != 0)
		{
			error_code = spin_res;
			//return spin_res;
		}
 
#ifdef NODE_INFO_LISTENER
		bool isNodeInfo = false; 
		for (std::uint8_t i = 1; i <= uavcan::NodeID::Max; i++)
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