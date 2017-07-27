#pragma once

#include "ToyWalker.h"

#include "Path.hpp"

namespace toywalker {

template <typename Value>
class PathSegment : public Path<Value> {
public:
	virtual Value beginning() = 0;
	virtual void beginning(Value const &) = 0;

	virtual Value end() = 0;
	virtual void end(Value const &) = 0;

	virtual void constraints(Value const & beginning,
	                         Value const & end,
				 Real duration)
	{
		this->beginning(beginning);
		this->end(end);
		this->duration(duration);
	}

};

}
