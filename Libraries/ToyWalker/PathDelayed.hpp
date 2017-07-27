#pragma once

#include "ToyWalker.h"

#include "Path.hpp"

namespace toywalker {

template <typename Value>
class PathDelayed : public Path<Value>
{
public:
	PathDelayed(Path<Value> & path, Real delay);

	virtual Value at(Real seconds);
	virtual Real duration() { return path().duration(); }
	virtual void duration(Real seconds) { path().duration(seconds); }

	Real delay() { return _delay; }
	void delay(Real seconds) { _delay = seconds; }

	Path<Value> & path() { return *_path; }
	void path(Path<Value> & path) { _path = &path; }

private:
	Path<Value> * _path;
	Real _delay;
};

}

// impl

namespace toywalker {

template <typename Value>
PathDelayed<Value>::PathDelayed(Path<Value> & path, Real delay)
: _path(&path), _delay(delay)
{ }

template <typename Value>
Value PathDelayed<Value>::at(Real seconds)
{
	return path().at(seconds - delay());
}

}
