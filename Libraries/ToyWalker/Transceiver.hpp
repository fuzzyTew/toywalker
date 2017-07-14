#pragma once

#include <string.h>

namespace toywalker {

template <typename... T>
class Transceiver
{
public:
	Transceiver(T &... vars)
	: vars{(void *)&vars...},
	  buflen(0)
	{
	}

	void receive()
	{
		for (;;)  {
			if (buflen == 0) {
				for (;;) {
					if (!readAvailable())
						return;
					read(buffer, 1);
					if (buffer[0] <= sizeof...(T)) {
						break;
					}
				}
				++ buflen;
			}
	
			unsigned sz = size(buffer[0]);
	
			if (buflen + readAvailable() < sz + 2)
				return;
	
			if (buflen < sz + 2)
				buflen += read(&buffer[buflen], sz + 2 - buflen);
	
			unsigned char checksum = 'A';
			for (unsigned char i = 1; i < sz + 1; ++ i) {
				checksum += buffer[i];
			}
	
			if (checksum != buffer[sz + 1]) {
				unsigned offset;
				
				for (offset = 1; offset < buflen; ++ offset) {
					//debug.write('H');
					if (buffer[offset] < sizeof...(T))
						break;
				}
	
				memmove(buffer, buffer + offset, buflen - offset);
				buflen -= offset;
			} else {
				memcpy(vars[buffer[0]], buffer + 1, sz);
				buflen -= sz + 2;
				received(buffer[0]);
			}
		}
	}

	void send(unsigned char index)
	{
		unsigned char sz = size(index);

		write(&index, 1);

		unsigned char checksum = 'A';
		for (unsigned char i = 0; i < sz; ++ i)
			checksum += static_cast<unsigned char *>(vars[index])[i];

		write(vars[index], sz);

		write(&checksum, 1);
	}

protected:
	virtual void received(unsigned char index) { }
	virtual unsigned readAvailable() = 0;
	virtual unsigned read(void * buffer, unsigned len) = 0;
	virtual void write(void * buffer, unsigned len) = 0;

private:
	template <int base = 0>
	static constexpr unsigned _maxSize()
	{
		return 0;
	}

	template <typename U, typename... V>
	static constexpr unsigned _maxSize()
	{
		return sizeof(U) > _maxSize<V...>() ? sizeof(U) : _maxSize<V...>();
	}

	static constexpr unsigned maxSize()
	{
		return _maxSize<T...>();
	}

	template <int base = 0>
	static constexpr unsigned _size(unsigned index)
	{
		return -1;
	}

	template <typename U, typename... V>
	static constexpr unsigned _size(unsigned index)
	{
		return (1 + sizeof...(V) + index == sizeof...(T)) ?
			sizeof(U) :
			_size<V...>(index);
	}

	static constexpr unsigned size(unsigned index) {
		return _size<T...>(index);
	}
	
	void * vars[sizeof...(T)];

	unsigned char buffer[maxSize() + 2];
	unsigned buflen;
};

}
