#pragma once

#include <string.h>

template <typename... T>
class TransceiverImplementation
{
public:
	TransceiverImplementation(T &... vars)
	: vars{(void *)&vars...}, buflen(0)
	{
	}

	void receive()
	{
		for (;;)  {
			int byte;
	
			if (buflen == 0) {
				for (;;) {
					if (!readAvailable())
						return;
					read(buffer, 1);
					if (buffer[0] <= sizeof...(T))
						break;
				}
				++ buflen;
			}
	
			unsigned char sz = size(buffer[0]);
	
			if (buflen + readAvailable() < sz + 2)
				return;
	
			buflen += read(&buffer[buflen], sz + 2 - buflen);
	
			unsigned char checksum = 0;
			for (unsigned char i = 1; i < sz + 1; ++ i)
				checksum += buffer[i];
	
			if (checksum != buffer[sz + 1]) {
				int offset;
				
				for (offset = 1; offset < buflen; ++ offset)
					if (buffer[offset] < sizeof...(T))
						break;
	
				memmove(buffer, buffer + offset, buflen -= offset);
			} else {
				memcpy(vars[buffer[0]], buffer + 1, sz);
			}
		}
	}

	void send(unsigned char index)
	{
		unsigned char sz = size(index);

		write(&index, 1);

		unsigned char checksum = 0;
		for (unsigned char i = 0; i < sz; ++ i)
			checksum += static_cast<unsigned char *>(vars[index])[i];

		write(vars, sz);

		write(&checksum, 1);
	}

protected:
	virtual unsigned readAvailable() = 0;
	virtual unsigned read(void * buffer, unsigned len) = 0;
	virtual void write(void * buffer, unsigned len) = 0;

private:
	constexpr unsigned _maxSize()
	{
		return 0;
	}

	template <typename U, typename... V>
	constexpr unsigned _maxSize()
	{
		return sizeof(U) > _maxSize<V...>() ? sizeof(U) : _maxSize<V...>();
	}

	constexpr unsigned maxSize()
	{
		return _maxSize<T...>();
	}

	constexpr unsigned _size(unsigned index)
	{
		return 0;
	}

	template <typename U, typename... V>
	constexpr unsigned _size(unsigned index)
	{
		if (1 + sizeof...(V) - index == sizeof...(T))
			return sizeof(U);
		else
			return _size<V...>(index);
	}

	unsigned size(unsigned index) {
		return _size<T...>(index);
	}
	
	void * vars[sizeof...(T)];

	unsigned char buffer[_maxSize<T...>() + 1];
	unsigned buflen;
};
