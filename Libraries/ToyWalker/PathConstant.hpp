#pragma once

#include "ToyWalker.h"

#include "Path.hpp"

namespace toywalker {

template <typename Value>
class PathConstant : public Path<Value> {
public:
	PathConstant(Value const & value);

	virtual Value at(Real seconds);

	Value const & value() { return _value; }
	void value(Value const & value) { _value = value; }

private:
	Value _value;
};

}

// impl

namespace toywalker {

template <typename Value>
PathConstant<Value>::PathConstant(Value const & value)
: _value(value)
{ }

template <typename Value>
Value PathConstant<Value>::at(Real seconds)
{
	return value();
}

}
