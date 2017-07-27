
#include "LimbIkFast.hpp"

namespace toywalker {

Vector3 LimbIkFast::footBody(Angles const & plan)
{
	Vector3 ret;
	computeFkTranslation3D(&plan(0), &ret(0), nullptr);
	return ret;
}

Limb::Angles LimbIkFast::planBody(Vector3 const & footDestination)
{
	IkFastSolutionArray<MAX_JOINTS> & solutions = IkFastSolutionArray<MAX_JOINTS>::instance;

	solutions.Clear();
	solutions.setDOF(servos());

	if (computeIkTranslation3D(&footDestination[0], nullptr, nullptr, solutions)) {

		Eigen::Array<Limb::Angles, Eigen::Dynamic, 1, 0, IkFastSolutionArray<MAX_JOINTS>::MAX_SOLUTIONS, 1> selectable;
	
		for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++ i)
		{
			size_t s;
			const ikfast::IkSolutionBase<Real> & curSolution = solutions.GetSolution(i);
			Angles curValues;
			
			curSolution.GetSolution(&curValues(0), NULL);

			for (std::size_t j = 0; j < servos(); ++ j) {
				Array2 limits = servo(j).angleLimit();
				if (curValues[j] < limits[0] || curValues[j] > limits[1]) {
					goto next;
				}
			}

			s = selectable.size();
			selectable.resize(s + 1);
			selectable[s].resize(servos());
			curSolution.GetSolution(&selectable[s][0], nullptr);
next:
			;
		}
		if (selectable.size()) {
			return selectable[selector.select(*this, &selectable[0], selectable.size())];
		}
	}
	return Angles((int)0);
}

}
