#pragma once

#include <Eigen/Geometry>

namespace toywalker {

	using Isometry2 = Eigen::Transform<Real, 2, Eigen::Isometry>;
	using Isometry3 = Eigen::Transform<Real, 3, Eigen::Isometry>;

	using Plane = Eigen::Hyperplane<Real, 3>;

	using Translation2 = Eigen::Translation<Real, 2>;
	using Translation3 = Eigen::Translation<Real, 3>;

	using AngleAxis = Eigen::AngleAxis<Real>;
}
