#pragma once

#include "ToyWalker.h"

#include "PathSegment.hpp"

#include "Vector.hpp"

namespace toywalker {

template <typename Value>
class PathSegmentQuadratic : public PathSegment<Value>
{
public:
	PathSegmentQuadratic(Value const & beginning,
	                     Value const & middle,
			     Real middleSeconds,
	                     Value const & end,
			     Real duration)
	: points(beginning.rows(), 3),
	  _middleSeconds(middleSeconds),
	  coeffs(beginning.rows(), 3)
	{
		Path<Value>::duration(duration);

		points.col(0) = beginning;
		points.col(1) = middle;
		points.col(2) = end;

		updateCoefficients();
	}

	Value at(Real seconds)
	{
		return (coeffs.col(0) * seconds + coeffs.col(1)) * seconds + coeffs.col(2);
	}

	Value beginning() { return points.col(0); }
	void beginning(Value const & value)
	{
		points.col(0) = value;
		updateCoefficients();
	}

	Value end() { return points.col(2); }
	void end(Value const & value)
	{
		points.col(2) = value;
		updateCoefficients();
	}

	virtual void constraints(Value const & beginning,
	                         Value const & end,
	                         Real duration)
	{
		points.col(0) = beginning;
		points.col(2) = end;
		this->duration(duration);
	}

	Value const & middle() { return points.col(1); }
	void middle(Value const & value)
	{
		points.col(1) = value;
		updateCoefficients();
	}

	void duration(Real seconds)
	{
		_middleSeconds *= seconds / Path<Value>::duration();
		Path<Value>::duration(seconds);
		updateCoefficients();
	}

	void constraints(Value const & beginning,
	                 Value const & middle,
	                 Real middleSeconds,
	                 Value const & end,
	                 Real duration)
	{
		points.col(0) = beginning;
		points.col(1) = middle;
		points.col(2) = end;
		_middleSeconds = middleSeconds;
		this->duration(duration);
	}


private:
	Eigen::Matrix<Real, Value::RowsAtCompileTime, 3, 0, Value::MaxRowsAtCompileTime, 3> points;
	Real _middleSeconds;
	Eigen::Array<Real, Value::RowsAtCompileTime, 3, 0, Value::MaxRowsAtCompileTime, 3> coeffs;

	void updateCoefficients()
	{
		Real _duration = Path<Value>::duration();

		coeffs = points * (Matrix3() <<
			0, _middleSeconds * _middleSeconds, _duration * _duration,
			0, _middleSeconds,                  _duration,
			1, 1,                               1
		).finished().inverse();
	}
};

}
