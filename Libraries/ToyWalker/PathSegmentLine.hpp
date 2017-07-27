#pragma once

#include "ToyWalker.h"

#include "PathSegment.hpp"

namespace toywalker {

template <typename Value>
class PathSegmentLine : public PathSegment<Value>
{
public:
	PathSegmentLine(Value const & beginning, Value const & end, Real duration);

	virtual Value at(Real seconds);

	virtual Value beginning() { return _beginning; }
	virtual void beginning(Value const & value) { _beginning = value; }

	virtual Value end() { return _end; }
	virtual void end(Value const & value) { _end = value; }

private:
	Value _beginning;
	Value _end;
};

}

// impl

namespace toywalker {

template <typename Value>
PathSegmentLine<Value>::PathSegmentLine(Value const & beginning, Value const & end, Real duration)
: _beginning(beginning), _end(end)
{
	this->duration(duration);
}

template <typename Value>
Value PathSegmentLine<Value>::at(Real seconds)
{
	return (end() - beginning()) * seconds / Path<Value>::duration() + beginning();
}

}
