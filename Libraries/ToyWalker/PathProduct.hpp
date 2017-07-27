#pragma once

#include "ToyWalker.h"

#include "Path.hpp"

namespace toywalker {

template <typename Value, size_t _MAX_FACTORS = 8>
class PathProduct : public Path<Value>
{
public:
	static constexpr size_t MAX_FACTORS = _MAX_FACTORS;

	template <typename... Paths>
	PathProduct(Paths &... paths);

	virtual Value at(Real seconds);
	virtual Real duration() { return path(0).duration(); }
	virtual void duration(Real seconds);

	size_t paths() const { return _paths.size(); }
	Path<Value> & path(size_t index) { return *_paths[index]; }

private:
	ArrayX<MAX_FACTORS, Path<Value>*> _paths;
};

}

// impl

namespace toywalker {

template <typename Value, size_t MAX_FACTORS>
template <typename... Paths>
PathProduct<Value, MAX_FACTORS>::PathProduct(Paths &... paths)
: _paths(ArrayX<MAX_FACTORS, Path<Value>*>::Map(((std::initializer_list<Path<Value> *>){&paths...}).begin(), sizeof...(paths)))
{ }

template <typename Value, size_t MAX_FACTORS>
Value PathProduct<Value, MAX_FACTORS>::at(Real seconds)
{
	Value product = path(0).at(seconds);

	for (size_t i = 1; i < paths(); ++ i) {
		product = product * path(i).at(seconds);
	}
	return product;
}

template <typename Value, size_t MAX_FACTORS>
void PathProduct<Value, MAX_FACTORS>::duration(Real seconds)
{
	for (int i = 0; i < _paths.size(); ++ i) {
		_paths[i]->duration(seconds);
	}
}

}
