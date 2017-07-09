#pragma once

#include <math.h>

namespace kinematics_ikfast_back_foot_translation3d {

	constexpr double xMin = -1.5 + 0.25;
	constexpr double xMax = -0.5 + 0.25;
	
	constexpr double yMin = -0.5;
	constexpr double yMax = 0.5;
	
	constexpr double zMin = -1.5;
	constexpr double zMax = -0.5;

}

namespace kinematics_ikfast_left_foot_translation3d {

	constexpr double xMin = 1.0 / 2 + 0.25 - 0.5;
	
	constexpr double xMax = 1.0 / 2 + 0.25 + 0.5;
	
	constexpr double yMin = sqrt(3) / 2 - 0.5;
	constexpr double yMax = sqrt(3) / 2 + 0.5;
	
	constexpr double zMin = -1.5;
	constexpr double zMax = -0.5;

}

namespace kinematics_ikfast_right_foot_translation3d {

	constexpr double xMin = 1.0 / 2 + 0.25 - 0.5;
	constexpr double xMax = 1.0 / 2 + 0.25 + 0.5;
	
	constexpr double yMin = -sqrt(3) / 2 - 0.5;
	constexpr double yMax = -sqrt(3) / 2 + 0.5;
	
	constexpr double zMin = -1.5;
	constexpr double zMax = -0.5;

}
