#include "uavcan_node.hpp"
#include "stm32f1xx_hal.h"

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

#include <config/config.hpp>
#include <config/config_storage.hpp>

#define ENUMERATION
#define PARAM_SERVER

namespace uavcan_node
{
	namespace
	{
		/*
		* Restart handler
		*/
		class RestartRequestHandler : public uavcan::IRestartRequestHandler
		{
		public:
			bool handleRestartRequest(uavcan::NodeID request_source) override
			{
				//os::lowsyslog("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
				//os::requestReboot();
				return true;
			}
		} restart_request_handler;
		
		#define NODE_ID 4
		#define NODE_NAME "io.px4.sapog" //"TestController"
		#define SELF_INDEX_DEFAULT 0//(NODE_ID - 1)
		#define SELF_DIR_DEFAULT 1

		static constexpr int RxQueueSize = 64;
		static constexpr uint32_t BitRate = 1000000;
		
		uavcan::Publisher<uavcan::equipment::esc::Status>* pub_status;
		uavcan::Publisher<uavcan::equipment::esc::RawCommand>* pub_raw_cmd; 
		
		os::config::Param<unsigned> param_node_id("uavcan_node_id", NODE_ID, 0, 125);
		os::config::Param<unsigned> param_esc_index("esc_index", SELF_INDEX_DEFAULT, 0, 15);
		os::config::Param<unsigned> param_cmd_ttl_ms("cmd_ttl_ms", 200, 100, 5000);
		os::config::Param<float> param_cmd_start_dc("cmd_start_dc", 1.0, 0.01, 1.0);
		os::config::Param<unsigned> param_light_index("light_index", 0,    0,    255);
		CONFIG_PARAM_INT("ctl_dir", SELF_DIR_DEFAULT, 0, 1)

		static int rpmCnt  = 0;
		static bool rpmUpdate = false, rpmIncrease = true;
		static int error_code = 0;
		static bool rawUpdate = false;
		static float raw_cmd_pct = 0.0f; 
		
		#define MOTOR_MAX_RPM (11.5F*980)
		#define MOTOR_MAX_CURRENT 15.0F 
		#define MOTOR_MIN_RPM 1000 	
		
		#define SIM_M_ROTATION_START (NODE_ID * 40)
		#define SIM_M_ROTATION_STOP (SIM_M_ROTATION_START + 20)
		unsigned self_index, self_dir; 
		int self_rpm = 0;
		float battery_voltage = 11.5F; 
		float motor_current = 0.001F;
		float motor_dc = 0.0F; 

		struct
		{
			size_t idx; 
			int data[8];
			bool is_updated;
		} raw_cmd = {0}; 	
		
		float motor_get_duty_cycle(void)
		{
			return motor_dc;
		}
		
		void motor_set_duty_cycle(int dc, int ttl_ms)
		{			
			//			motor_dc = 	
			//self_rpm = (int)((MOTOR_MAX_RPM - MOTOR_MIN_RPM)*abs(dc));
		}
		
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

		void cb_10Hz(const uavcan::TimerEvent& event)
		{
			//#ifdef true
			static bool transmit = false; 
			uavcan::equipment::esc::Status msg;
		
			/*if (msg.rpm.size() <= self_index) {
				return;
			}

			rpm = msg.rpm[self_index];*/
			
			//msg.esc_index = self_index;
			//msg.rpm = motor_get_rpm();
			//motor_get_input_voltage_current(&msg.voltage, &msg.current);
			//msg.power_rating_pct = static_cast<unsigned>(motor_get_duty_cycle() * 100 + 0.5F);
			//msg.error_count = motor_get_zc_failures_since_start();

			//msg.temperature = float(temperature_sensor::get_temperature_K());
			//if (msg.temperature < 0) {
			//	msg.temperature = std::numeric_limits<float>::quiet_NaN();
			//}

			//if (motor_is_idle()) {
			//	// Lower the publish rate to 1Hz if the motor is not running
			//	static uavcan::MonotonicTime prev_pub_ts;
			//	if ((event.scheduled_time - prev_pub_ts).toMSec() >= 990) {
			//		prev_pub_ts = event.scheduled_time;
			//		pub_status->broadcast(msg);
			//	}
			//}
			//else {
			//	pub_status->broadcast(msg);
			//}
			

			msg.esc_index = self_index;
			msg.rpm = self_rpm;
//			msg.voltage = battery_voltage;
//			msg.current = motor_current;
//			msg.temperature = 273.15F + 24.0F;
//			msg.power_rating_pct = static_cast<unsigned>(motor_get_duty_cycle() * 100 + 0.5F);
		    msg.voltage = 3.3F;
			msg.current = 0.001F;
			msg.temperature = 24.0F;
			msg.power_rating_pct = static_cast<unsigned>(.5F * 100 + 0.5F);
			msg.error_count = 0;

			int res = pub_status->broadcast(msg);
			if (res < 0) {
				error_code = res;
			}	
			
//			uavcan::equipment::esc::RawCommand raw_msg; 
//			for (unsigned i = 0; i < 4; i++) {
//				float scaled = -100.0F;
//				raw_msg.cmd.push_back(static_cast<int>(scaled));
//			}
//			res = pub_raw_cmd->broadcast(raw_msg);
//			if (res < 0) {
//				error_code = res;
//			}	
			//#endif 
		}
		
//		void cb_esc_status_command(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg)
//		{
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);		
//		}
		
		bool is_equal(float x, float y) {
			//return std::fabs(x - y) < std::numeric_limits<double>::epsilon();
			return std::fabs(x - y) < 0.0001;
		}

		float trim(float v)
		{
			if (v < 0.0f) return 0.0f;
			if (v > 1.0f) return 1.0f; 
			return v; 
		}
		int raw_value;
		
		void cb_raw_command(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg)
		{
			if (msg.cmd.size() <= self_index)
			{
				if (!rawUpdate)
					rawUpdate = true;
					
				//raw_cmd_pct = 0.0f; 
				raw_value = 0; 
				//motor_stop();
				return;
			} 
			
			raw_value = msg.cmd[self_index];
			rawUpdate = true;
			
//			size_t size = msg.cmd.size(); 
//			if (size < 8)
//			{
//				raw_cmd.idx = size; 
//				for (int i = 0; i < size; i++)
//				{
//					raw_cmd.data[i] = msg.cmd[i];	
//				}
//				raw_cmd.is_updated = true; 
//			}
						
			// scaled_dc может принимать значения -1.0 ... 1.0 
//			const float scaled_dc =
//			msg.cmd[self_index] / float(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max());
//
//			//motor_set_duty_cycle(scaled_dc, 0);
//			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);		
//			if (is_equal(0.0f, scaled_dc)) // чтобы исключить потерю сообщения с нулевыми значениями шим
//			{
//				raw_cmd_pct = 0.0f;
//				raw_value = 0;
//				rawUpdate = true;
//			} else if (!is_equal(raw_cmd_pct, scaled_dc))
//			{				
//				raw_cmd_pct = trim(scaled_dc); 
//				raw_value = msg.cmd[self_index];
//				rawUpdate = true;				
//			}
//			else
//			{
//				rawUpdate = false;
//			}
//			const bool idle = motor_is_idle();
//			const bool accept = (!idle) || (idle && (scaled_dc <= max_dc_to_start));
//
//			if (accept && (scaled_dc > 0)) {
//				motor_set_duty_cycle(scaled_dc, command_ttl_ms);
//			}
//			else {
//				motor_stop();
//			}
		}

		void cb_rpm_command(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RPMCommand>& msg)
		{
			//led_red_toggle(); 
			if (msg.rpm.size() <= self_index) {
				//motor_stop();
				return;
			}

			const int rpm = msg.rpm[self_index];

			if (rpm > 0) {
				//motor_set_rpm(rpm, command_ttl_ms);				
			}
			else {
				//motor_stop();				
			}
			
			if (self_rpm != rpm)
			{
				self_rpm = msg.rpm[0];
				rpmUpdate = true;				
			}
			else
			{
				rpmUpdate = false;
			}
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
	
		enum motor_forced_rotation_direction
		{
			MOTOR_FORCED_ROTATION_NONE,
			MOTOR_FORCED_ROTATION_FORWARD,
			MOTOR_FORCED_ROTATION_REVERSE,
		};
		
		bool motor_is_idle(void) {
			return true; 
		}

		enum motor_forced_rotation_direction motor_get_forced_rotation_direction(void)
		{
			static int cnt = 0; 
			cnt++; 
			if (cnt > SIM_M_ROTATION_START && cnt < SIM_M_ROTATION_STOP)
			{
				if (NODE_ID == 1 || NODE_ID == 2)
				{
					return MOTOR_FORCED_ROTATION_FORWARD ; 					
				}
				else if (NODE_ID == 3 || NODE_ID == 4)
				{
					return MOTOR_FORCED_ROTATION_REVERSE ;
				}
			}				
		
			return MOTOR_FORCED_ROTATION_NONE ; 
		};
		
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
					if (strcmp(name.c_str(), dir) == 0)
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
				void (EnumerationHandler::*)
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

				if (event.real_time >= confirmation_deadline_) {
					//os::lowsyslog("UAVCAN: Enumeration request expired\n");
					this->stop();
					//led_ctl.unset();
				}
				else {
					const auto rotation = motor_get_forced_rotation_direction();
					//const auto rotation = MOTOR_FORCED_ROTATION_FORWARD;
					if (rotation != MOTOR_FORCED_ROTATION_NONE) {
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
				if (!req.parameter_name.empty() && (req.parameter_name != "esc_index"))
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
			{ }

			int start()
			{
				return srv_.start(CallbackBinder(this, &EnumerationHandler::handleBegin));
			}
		};
		
		uavcan::LazyConstructor<EnumerationHandler> enumeration_handler_;		
#endif

	}
	
	void testRawCmd(void)
	{	
		uavcan::equipment::esc::RawCommand msg;
		float scaled = 5000.0; 	
		msg.cmd.push_back(static_cast<int>(scaled));
		msg.cmd.push_back(static_cast<int>(scaled));
		msg.cmd.push_back(static_cast<int>(scaled));
		msg.cmd.push_back(static_cast<int>(scaled));
	
		size_t size = msg.cmd.size(); 
		if (size < 8)
		{
			raw_cmd.idx = size; 
			for (int i = 0; i < size; i++)
			{
				raw_cmd.data[i] = msg.cmd[i];	
			}
			raw_cmd.is_updated = true; 
		}
	}
	
	bool rawArrayIsUpd(void)
	{
		return raw_cmd.is_updated;  
	}
		
	void getRawCmd(int* p, size_t* idx)
	{
		for (int i = 0; i < raw_cmd.idx; i++) p[i] = raw_cmd.data[i]; 
		*idx = raw_cmd.idx;
		raw_cmd.is_updated = false; 
	}

	bool getRPMUpdate() 
	{
		return rpmUpdate;
	}

	int getRPM() {

		return self_rpm;
	}	
	
	uint32_t last_tick = 0;
	bool raw_increase = true; 
	
	bool getRawUpdate(void) 
	{		 
		return rawUpdate;
	}

	bool getRaw(float* raw)
	{		
		if(rawUpdate) {			
			rawUpdate = false; 
			*raw = raw_cmd_pct; 
			return true; 
		}
		return false;
	}
	
	bool getIntRaw(int* raw)
	{
		if (!rawUpdate)
			return false;
			
		rawUpdate = false; 
		*raw = raw_value; 
		return true; 
		
		// ToDo для теста
		//*raw = 1; 
		//return true;
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

		uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
		sw_version.major = 2;
		sw_version.minor = 0;
		sw_version.vcs_commit = 0xb1354f4; // git rev-parse --short HEAD
		sw_version.image_crc = 0x0102030405060708; // image CRC-64-WE see https://uavcan.org/Specification/7._List_of_standard_data_types/ 
		sw_version.optional_field_flags |= sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
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
	
		mynode.setRestartRequestHandler(&restart_request_handler);
//		int res = get_begin_firmware_update_server().start(&handle_begin_firmware_update_request);		
//		if (res < 0)
//		{
//			led_red_toggle(); //board::die(res);
//		}
		
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

		mynode.setModeOperational();	
		
		// Config
		static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
		const int config_init_res = os::config::init(&config_storage_backend);
		if (config_init_res < 0)
		{
			error_code = -1; //die(config_init_res);
		}
		self_index = param_esc_index.get();

		return 0; 
	}

	int NodeOnePub() {

		int res = 0;

		//uavcan::ServiceClient <uavcan::protocol::GetNodeInfo, GetNodeInfoCallback> client(getNode());
		//const int client_init_res = client.init();
		//if (client_init_res < 0)
		//{
		//	res = -1; 
		//	//throw std::runtime_error("Failed to init the client; error: " + std::to_string(client_init_res));
		//}

		//client.setCallback([](const uavcan::ServiceCallResult<GetNodeInfo>& call_result)
		//{
		//	//if (call_result.isSuccessful())  // Whether the call was successful, i.e. whether the response was received
		//	//{
		//	//	// The result can be directly streamed; the output will be formatted in human-readable YAML.
		//	//	std::cout << call_result << std::endl;
		//	//}
		//	//else
		//	//{
		//	//	std::cerr << "Service call to node "
		//	//		<< static_cast<int>(call_result.getCallID().server_node_id.get())
		//	//		<< " has failed" << std::endl;
		//	//}
		//});

		//client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(200));

		//client.setPriority(uavcan::TransferPriority::OneHigherThanLowest);

		//uavcan::protocol::GetNodeInfo::Request req;
		//
		//const int call_res = client.call(NODE_ID, req);
		//if (call_res < 0)
		//{
		//	res = -2; 
		//	//throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
		//}

		//pub_status = new uavcan::Publisher<uavcan::equipment::esc::Status>(getNode());
		//res = pub_status->init();
		//if (res != 0) {
		//	error_code = res;
		//	return res;
		//}

		return res;
	}		

	int NodeStartPub() {
			
		int res = 0;

		static uavcan::Timer timer_10hz(getNode());
		

		pub_status = new uavcan::Publisher<uavcan::equipment::esc::Status>(getNode());
		res = pub_status->init();
		if (res != 0) {
			error_code = res;
			return res;
		}
		
//		pub_raw_cmd = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(getNode());
//		res = pub_raw_cmd->init();
//		if (res != 0) {
//			error_code = res;
//			return res;
//		}
		
		timer_10hz.setCallback(&cb_10Hz);
		timer_10hz.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));
	}

	int NodeStartSub() {

		static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(getNode());
		static uavcan::Subscriber<uavcan::equipment::esc::RPMCommand> sub_rpm_command(getNode());
		static uavcan::Subscriber<uavcan::equipment::indication::BeepCommand> sub_beep_command(getNode());
		static uavcan::Subscriber<uavcan::equipment::indication::LightsCommand> sub_light_command(getNode());
		//static uavcan::Subscriber<uavcan::protocol::dynamic_node_id::Allocation> sub_dynId_command(getNode());
		//static uavcan::Subscriber<uavcan::protocol::NodeStatus> sub_ns_command(getNode());
//		static uavcan::Subscriber<uavcan::equipment::esc::Status> sub_esc_status(getNode());

		int res = 0;
				
//		res = sub_esc_status.start(cb_esc_status_command);
//		if (res != 0) {
//			return res;
//		}

		res = sub_raw_command.start(cb_raw_command);
		if (res != 0) {
			return res;
		}

		res = sub_rpm_command.start(cb_rpm_command);
		if (res != 0) {
			return res;
		}

		res = sub_beep_command.start(cb_beep_command);
		if (res != 0) {
			return res;
		}

		res = sub_light_command.start(cb_light_command);
		if (res != 0) {
			return res;
		}

//		res = sub_dynId_command.start(cb_dynId_command);
//		if (res != 0) {
//			return res;
//		}
			
//		res = sub_ns_command.start(cb_ns_command);
//		if (res != 0) {
//			return res;
//		}
	}

	int NodeSpin()
	{		
		const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
		if (spin_res != 0)
		{
			error_code = spin_res;
			//return spin_res;
		}	
		return 0; 
	}
}