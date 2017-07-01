#pragma once

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"

#include <stlport.h>
#include <stlport-missing.h>
#include <Eigen31.h>

// remove exceptions from all code
#define throw

// remove streams
#define IKFAST_ASSERT(b) {}

// ikfast.h is modified to not require vectors if this is defined
#define HAVE_NO_VECTOR
