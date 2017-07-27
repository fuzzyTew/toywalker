#pragma once

#include "ToyWalker.h"

#include "Path.hpp"

#include "Vector.hpp"

namespace toywalker {

template <typename Value>
class PathSequence : public Path<Value>
{
public:
	static constexpr size_t MAX_SUBPATHS = 8;

	template <typename... Paths>
	PathSequence(Paths &... paths);

	virtual Value at(Real seconds);
	virtual Real duration();

	size_t paths() const { return _paths.size(); }
	Path<Value> & path(size_t index) { return *_paths[index]; }

private:
	ArrayX<MAX_SUBPATHS, Path<Value>*> _paths;

	int currentIndex;
	Real currentStart;
};

}

// impl

namespace toywalker {

template <typename Value>
template <typename... Paths>
PathSequence<Value>::PathSequence(Paths &... paths)
: _paths(ArrayX<MAX_SUBPATHS, Path<Value>*>::Map(std::initializer_list<Path<Value> *>({&paths...}).begin(), sizeof...(paths))),
  currentIndex(0),
  currentStart(0)
{ }

template <typename Value>
Value PathSequence<Value>::at(Real seconds)
{
	while (seconds > currentStart + _paths[currentIndex]->duration() && currentIndex < _paths.size() - 1) {
		currentStart += _paths[currentIndex]->duration();
		++ currentIndex;
	}
	if (currentIndex > 0) {
		while (seconds < currentStart) {
			-- currentIndex;
			if (currentIndex == 0) {
				currentStart = 0;
				break;
			} else {
				currentStart -= _paths[currentIndex]->duration();
			}
		}
	}
	return _paths[currentIndex]->at(seconds - currentStart);
}

template <typename Value>
Real PathSequence<Value>::duration()
{
	Real sum = 0;
	for (int i = 0; i < _paths.size(); ++ i)
		sum += _paths[i]->duration();
	return sum;
}

}
