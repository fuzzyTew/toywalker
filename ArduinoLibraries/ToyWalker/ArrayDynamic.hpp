#pragma once

#include "StdLib.hpp"

template <typename T>
class ArrayDynamic
{
public:
	template <typename... U>
	ArrayDynamic(U... initializers)
	: len(sizeof...(initializers)),
	  data(new T[len] {initializers...})
	{ }

	~ArrayDynamic()
	{
		clear();
	}

	T & operator[](size_t i)
	{
		return data[i];
	}

	T const & operator[](size_t i) const
	{
		return data[i];
	}

	size_t size() const
	{
		return len;
	}

	void append(T && object)
	{
		T * data2 = reinterpret_cast<T *>(new char[(len + 1) * sizeof(T)]);

		for (size_t i = 0; i < len; ++ i)
			new(data2 + i) T(static_cast<T &&>(data[i]));
		new(data2 + len) T(object);

		delete [] data;

		data = data2;
		len ++;
	}

	T && erase(size_t i)
	{
		T * data2 = reinterpret_cast<T *>(new char[(len - 1) * sizeof(T)]);
		size_t j;

		for (j = 0; j < i; ++ j)
			new(data2 + j) T(static_cast<T &&>(data[j]));

		for (j = i; j < len - 1; ++ j)
			new(data2 + j) T(static_cast<T &&>(data[j + 1]));

		T && ret = static_cast<T &&>(data[i]);

		delete [] data;

		data = data2;
		len --;

		return static_cast<T &&>(ret);
	}

	void clear()
	{
		delete [] data;
		len = 0;
	}

private:
	size_t len;
	T * data;
};
