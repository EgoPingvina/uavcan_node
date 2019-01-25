#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>

namespace uavcan_node {

	int configureNode();	

	int NodeSpin(); 

	int NodeStartPub();

	int NodeStartSub();

	bool getIntRaw(int* raw);	

	unsigned self_esc_index(void); 
}
