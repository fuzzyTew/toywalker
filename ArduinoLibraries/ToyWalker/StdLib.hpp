#pragma once

#include <stddef.h>

inline void * operator new(size_t, void* where)
{
	return where;
}
