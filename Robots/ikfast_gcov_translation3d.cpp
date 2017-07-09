#include <ikfast.h>

#include <iostream>

using namespace IKFAST_NAMESPACE;

int main()
{
	IkReal limits[6] = {xMin, xMax, yMin, yMax, zMin, zMax};
	IkReal pos[3];
	ikfast::IkSolutionList<IkReal> solutions;
	int total = 0;
	int success = 0;
	int maxSolutions = 0;
	for (int ix = 0; ix <= DECIMATION; ++ ix)
		for (int iy = 0; iy <= DECIMATION; ++ iy)
			for (int iz = 0; iz <= DECIMATION; ++ iz) {
				pos[0] = limits[0] + (limits[1] - limits[0]) * ix / IkReal(DECIMATION);
				pos[1] = limits[2] + (limits[3] - limits[2]) * ix / IkReal(DECIMATION);
				pos[2] = limits[4] + (limits[5] - limits[4]) * ix / IkReal(DECIMATION);
				++ total;
				if (ComputeIk(&pos[0], nullptr, nullptr, solutions)) {
					++ success;
					if (solutions.GetNumSolutions() > maxSolutions)
						maxSolutions = solutions.GetNumSolutions();
				}

			}
	std::cout << (success * 100 / total) << "% destinations reachable.  Max solution count: " << maxSolutions << std::endl;

	IkReal dummyJoints[64];
	ComputeFk(dummyJoints, &pos[0], nullptr);

	return 0;
}
