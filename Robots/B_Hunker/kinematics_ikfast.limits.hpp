#pragma once

#include <math.h>

namespace kinematics {
	constexpr double home = 0.72;
	constexpr double reach = 11.5 * 0.06;
}

namespace kinematics_ikfast_backRightFoot_translation3d {
	using namespace kinematics;

	constexpr double xMin = -home - reach;
	constexpr double xMax = -home + reach;
	
	constexpr double yMin = -reach;
	constexpr double yMax = reach;
	
	constexpr double zMin = 0.15 - reach;
	constexpr double zMax = 0.15 + reach;
}

namespace kinematics_ikfast_backLeftFoot_translation3d {
	using namespace kinematics;

	constexpr double xMin = -reach;
	constexpr double xMax = reach;
	
	constexpr double yMin = home - reach;
	constexpr double yMax = home + reach;
	
	constexpr double zMin = 0.15 - reach;
	constexpr double zMax = 0.15 + reach;
}

namespace kinematics_ikfast_frontRightFoot_translation3d {
	using namespace kinematics;

	constexpr double xMin = -reach;
	constexpr double xMax = reach;
	
	constexpr double yMin = -home - reach;
	constexpr double yMax = -home + reach;
	
	constexpr double zMin = 0.15 - reach;
	constexpr double zMax = 0.15 + reach;
}

namespace kinematics_ikfast_frontLeftFoot_translation3d {
	using namespace kinematics;

	constexpr double xMin = home - reach;
	constexpr double xMax = home + reach;
	
	constexpr double yMin = -reach;
	constexpr double yMax = reach;
	
	constexpr double zMin = 0.15 - reach;
	constexpr double zMax = 0.15 + reach;
}
