#pragma once

#include <uavcan_stm32/uavcan_stm32.hpp>

namespace uavcan_node
{
	int configureNode();	

	/// <summary>
	/// Spinning for n second.
	/// The method spin() may return earlier if an error occurs(e.g.driver failure).
	/// All error codes are listed in the header uavcan / error.hpp.
	/// </summary>
	int NodeSpin(); 

	int NodeStartPub();

	int NodeStartSub();

	int NodeOnePub();

	bool getRPMUpdate();

	int getRPM();
	
	bool getRawUpdate(void);
	
	bool getRaw(float* raw);
	
	bool getIntRaw(int* raw); 
	
	bool rawArrayIsUpd(void);
		
	void getRawCmd(int* p, size_t* idx);
	
	void testRawCmd(void); 
}