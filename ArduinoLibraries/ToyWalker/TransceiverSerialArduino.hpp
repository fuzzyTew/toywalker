#pragma once

#include "TransceiverImplementation.hpp"

template <typename... T>
class TransceiverSerialArduino : public TransceiverImplementation<T...>
{
public:
	TransceiverSerialArduino(T &... vars)
	: TransceiverImplementation<T...>(vars...)
	{
		Serial.begin(115200);
	}

protected:
	unsigned readAvailable()
	{
		return Serial.available();
	}

	unsigned read(void * buffer, unsigned len)
	{
		return Serial.readBytes(buffer, len);
	}

	void write(void * buffer, unsigned len)
	{
		Serial.write(buffer, len);
	}
};
