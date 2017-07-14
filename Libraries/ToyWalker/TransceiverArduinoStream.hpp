#pragma once

#include "TransceiverImplementation.hpp"

#include <Stream.h>

namespace toywalker {

template <typename... T>
class TransceiverArduinoStream : public TransceiverImplementation<T...>
{
public:
	TransceiverArduinoStream(Stream & stream, T &... vars)
	: TransceiverImplementation<T...>(vars...),
	  stream(stream)
	{ }

private:
	unsigned readAvailable()
	{
		return stream.available();
	}

	unsigned read(void * buffer, unsigned len)
	{
		return stream.readBytes((char *)buffer, (size_t)len);
	}

	void write(void * buffer, unsigned len)
	{
		stream.write((uint8_t *)buffer, (size_t)len);
	}

	Stream & stream;
};

}
