// hack around stlport bug of multiply including stdexcept implementation when iostreams disabled by including it before disabling
#include <stdexcept>

// don't include streams!
#define _STLP_NO_IOSTREAMS
#define _STLP_IOSFWD
#define EIGEN_IO_H
#define FUSION_SEQUENCE_IO_10032005_0836

#include <boost_1_51_0.h>

// forward is used by eigen
#include <boost/move/move.hpp>
namespace std {
	template <class T, class U>
	inline T&& forward(U&& t)
	{
		return static_cast<T&&>(boost::forward(t));
	}
}

// isfinite is used by ikfast
#include <boost/math/special_functions/fpclassify.hpp>
namespace std {
	template <class T> inline bool isfinite(T x) { return boost::math::isfinite(x); }
}
