#pragma once

#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/enumeration/Indication.hpp>
#include <uavcan/protocol/param_server.hpp>

#include <config/config_storage.hpp>

#include "CanNode.hpp"

namespace Controllers
{	
	#define NODE_NAME				"io.px4.rotor"
	
	#define SELF_DIR_DEFAULT		1	
	#define SIM_M_ROTATION_START	(nodeId * 40)
	#define SIM_M_ROTATION_STOP		(SIM_M_ROTATION_START + 20)

	CONFIG_PARAM_INT("ctl_dir", SELF_DIR_DEFAULT, 0, 1)

	class ESCController : public CanNode
	{
	public:
		ESCController();
	
		void Initialize() override;

		void ConfigureNode() override;
	
		bool GetValue(int32_t* raw) override;
		
		inline void Output() override;

	private:
		#define ENUMERATION
		#define PARAM_SERVER
	
		typedef uavcan::MethodBinder<ESCController*,
			void(ESCController::*)(const uavcan::TimerEvent&)>
			    StatusCallbackBinder;
	
		typedef uavcan::MethodBinder<ESCController*,
			void(ESCController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>&)>
			    RawCommandCallbackBinder;

		static constexpr unsigned configStorageSize		= 1024;

		void* const configStorageAddress;
	
		/// <summary>
		/// Value from CAN for this ESC
		/// </summary>
		int32_t value;

		unsigned selfIndex, selfDirection;
	
		uavcan::Publisher<uavcan::equipment::esc::Status>* statusPublisher;

		os::config::Param<unsigned> paramESCIndex;

		/// <summary>
		/// Uses by timer for sending Status with some period
		/// </summary>
		void StatusCallback(const uavcan::TimerEvent& event);
	
		/// <summary>
		/// Callback of new RawCommand package
		/// </summary>
		void RawCommandCallback(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg);

		#ifdef PARAM_SERVER

		uavcan::ParamServer& get_param_server()
		{
			static uavcan::ParamServer server(this->GetNode());
			return server;
		}

		/*
		 * Param access server
		 */
		class ParamManager : public uavcan::IParamManager
		{
		public:
			ParamManager(unsigned *index, unsigned *direction)
			{
				this->index		= index;
				this->direction = direction;
			}
		
		private:		
			unsigned *index, *direction;
		
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
				if (name != nullptr)
					out_name = name;
			}
		
			void assignParamValue(const Name& name, const Value& value) override
			{
				Value v = value;
				float native_value = 0.0f;

				if (v.is(uavcan::protocol::param::Value::Tag::boolean_value))
					native_value = v.to<uavcan::protocol::param::Value::Tag::boolean_value>() ? 1.0f : 0.0f;
				else if (v.is(uavcan::protocol::param::Value::Tag::integer_value))
				{
					native_value = v.to<uavcan::protocol::param::Value::Tag::integer_value>();
				
					const char ind[] = { "esc_index" };
					if (strcmp(name.c_str(), ind) == 0)
						*index = value.integer_value;
				
					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
				}
				else if (value.is(uavcan::protocol::param::Value::Tag::real_value))
					native_value = v.to<uavcan::protocol::param::Value::Tag::real_value>();

				(void)configSet(name.c_str(), native_value);
			}

			void readParamValue(const Name& name, Value& out_value) const override
			{
				ConfigParam descr;
				const int32_t isOk = configGetDescr(name.c_str(), &descr);
				if (isOk >= 0)
				{
					convert(configGet(name.c_str()), descr.type, out_value);
					const char dir[] = { "ctl_dir" };
				
					if (strcmp(name.c_str(), dir) == 0)
					{
						convert(*direction, CONFIG_TYPE_INT, out_value);
					}
				}
			}

			void readParamDefaultMaxMin(const Name& name, Value& out_default, NumericValue& out_max, NumericValue& out_min) const override
			{
				ConfigParam descr;
				const int32_t isOk = configGetDescr(name.c_str(), &descr);
				if (isOk >= 0)
				{
					convert(descr.default_, descr.type, out_default);
					convertNumeric(descr.max, descr.type, out_max);
					convertNumeric(descr.min, descr.type, out_min);
				}
			}

			int saveAllParams() override
			{
				return configSave();
			}

			int eraseAllParams() override
			{
				return configErase();
			}
		};
	
		ParamManager paramManager;

		#endif 		

		#ifdef ENUMERATION		
		/*
		* Enumeration handler
		*/
		class EnumerationHandler : public uavcan::TimerBase
		{
			static constexpr int32_t CONFIRMATION_CHECK_INTERVAL_MSEC = 50;

			typedef uavcan::MethodBinder<EnumerationHandler*,
				void(EnumerationHandler::*)
				(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>&,
				uavcan::protocol::enumeration::Begin::Response&)>
			CallbackBinder;

			uavcan::ServiceServer<uavcan::protocol::enumeration::Begin, CallbackBinder> srv_;
			uavcan::Publisher<uavcan::protocol::enumeration::Indication> pub_;
			uavcan::MonotonicTime confirmation_deadline_;

			void finish(bool reverse)
			{
				uavcan::protocol::enumeration::Indication msg;
				msg.parameter_name = "esc_index";
				pub_.broadcast(msg);

				(void)configSet("ctl_dir", reverse ? 1 : 0);
			}

			enum motor_forced_rotation_direction
			{
				MOTOR_FORCED_ROTATION_NONE,
				MOTOR_FORCED_ROTATION_FORWARD,
				MOTOR_FORCED_ROTATION_REVERSE,
			};

			void handleTimerEvent(const uavcan::TimerEvent& event) override
			{
				if (event.real_time >= confirmation_deadline_)
				{
					this->stop();
					return;
				}
			
				#pragma region user motors numbering simulation
			
				static int32_t cnt = 0;
				cnt++;
				
				if (!(cnt > SIM_M_ROTATION_START && cnt < SIM_M_ROTATION_STOP))
					return;
			
					#pragma endregion
				
				int32_t rotation;
				
				if (nodeId == 1 || nodeId == 2)
					rotation = MOTOR_FORCED_ROTATION_FORWARD;
				else if (nodeId == 3 || nodeId == 4)
					rotation = MOTOR_FORCED_ROTATION_REVERSE;
				
				if (rotation != MOTOR_FORCED_ROTATION_NONE)
				{
					const bool reverse = rotation != MOTOR_FORCED_ROTATION_FORWARD;
					finish(reverse);
				}
			}

			void handleBegin(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>& req,
				uavcan::protocol::enumeration::Begin::Response& resp)
			{
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
					resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_PARAMETER;
					return;
				}

				confirmation_deadline_ = req.getMonotonicTimestamp() + uavcan::MonotonicDuration::fromMSec(req.timeout_sec * 1000);
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

			int32_t start()
			{
				return srv_.start(CallbackBinder(this, &EnumerationHandler::handleBegin));
			}
		};

		uavcan::LazyConstructor<EnumerationHandler> enumeration_handler_;
		#endif

	};
}