#include "CanNode.hpp"

using namespace Controllers;

UniqueID CanNode::ReadUID() const
{
	UniqueID uid;

	// Unique device ID register has address 0x1FFFF7E8
	std::memcpy(uid.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);

	return uid;
}

uavcan::ICanDriver& CanNode::GetCanDriver()
{
	static uavcan_stm32::CanInitHelper<rxQueueSize> can;

	static bool initialized = false;

	if (!initialized)
	{
		initialized = true;
		if (can.init(bitRate) != 0)
		{
			// ToDo init error uavcan::ErrDriver;
			this->ErrorHandler(__LINE__);
		}
	}

	return can.driver;
}

Node& CanNode::GetNode()
{
	static Node mynode(this->GetCanDriver(), uavcan_stm32::SystemClock::instance());
	return mynode;
}

void CanNode::NodeSpin() 
{
	if (this->GetNode().spin(uavcan::MonotonicDuration::fromMSec(100)) != 0)
		this->ErrorHandler(__LINE__);
}
	
unsigned CanNode::SelfIndex() const
{
	return this->selfIndex;
}

bool CanNode::IsValueUpdate(void) const
{		 
	return this->isValueUpdate;
}