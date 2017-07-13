#pragma once

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"

#ifdef __AVR__
	#include <stlport.h>
	#include <stlport_missing.h>
#elif defined(__arm__)
	#if defined(__io)
		#undef __io
	#endif
#endif
#define EIGEN_DONT_ALIGN
#include <Eigen31.h>

// remove exceptions from all code
#define throw

// remove streams
#define IKFAST_ASSERT(b) {}

// ikfast.h is modified to not require vectors if this is defined
#define HAVE_NO_VECTOR
