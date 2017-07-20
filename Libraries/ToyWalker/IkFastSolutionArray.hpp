#pragma once

#include "ToyWalker.h"

#include "ikfast.h"

#include <Eigen/Core>

namespace toywalker {

template <size_t MAX_JOINTS, size_t _MAX_SOLUTIONS=8>
class IkFastSolutionArray : public ikfast::IkSolutionListBase<Real>
{
public:
	static constexpr size_t MAX_SOLUTIONS = _MAX_SOLUTIONS;
	IkFastSolutionArray(size_t dof = 0)
	: dof(dof)
	{ }
	virtual ~IkFastSolutionArray() {}

	virtual size_t AddSolution(const ikfast::IkSingleDOFSolutionBase<Real> * vinfos, const int * vfree)
	{
		if (solutions.size() < MAX_SOLUTIONS) {
			size_t i = solutions.size();
			solutions.resize(i + 1);
			solutions[i].vinfos.resize(dof);
			memcpy(&solutions[i].vinfos[0], vinfos, dof * sizeof(*vinfos));
			return i;
		} else {
			return numSolutions - 1;
		}
	}
	virtual const ikfast::IkSolutionBase<Real>& GetSolution(size_t index) const
	{
		return solutions[index];
	}
	virtual size_t GetNumSolutions() const
	{
		return solutions.size();
	}
	virtual void Clear()
	{
		solutions.resize(0);
	}

	void setDOF(size_t dof)
	{
		this->dof = dof;
	}

	static IkFastSolutionArray<MAX_JOINTS,MAX_SOLUTIONS> instance;

private:
	size_t numSolutions;
	struct Solution : public ikfast::IkSolutionBase<Real>
	{
		void GetSolution(Real * solution, const Real * freevalues) const
		{
			for (std::size_t i = 0; i < vinfos.size(); ++ i)
				if (vinfos[i].freeind < 0) {
					solution[i] = vinfos[i].foffset;
				} else {
					solution[i] = freevalues[vinfos[i].freeind] * vinfos[i].fmul + vinfos[i].foffset;
					if (solution[i] > M_PI)
						solution[i] -= 2 * M_PI;
					else if (solution[i] < -M_PI)
						solution[i] += 2 * M_PI;
				}

		}

		virtual const int * GetFree() const
		{
			return nullptr;
		}

		virtual const int GetDOF() const
		{
			return vinfos.size();
		}

		Eigen::Array<ikfast::IkSingleDOFSolutionBase<Real>, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> vinfos;
	};

	Eigen::Array<Solution, Eigen::Dynamic, 1, 0, MAX_SOLUTIONS, 1> solutions;
	size_t dof;
};

}
