#include <stdexcept>

#include <math.h>

#include <stl/_string.h>

namespace std {

#include <stl/_range_errors.c>
	
void __stl_throw_length_error(const char *) {}
void __stl_throw_out_of_range(const char *) {}

#include <stl/_stdexcept_base.c>

exception::exception() {}
exception::~exception() {}

runtime_error::~runtime_error() {}

}
