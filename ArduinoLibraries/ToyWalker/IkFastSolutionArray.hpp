#pragma once

#include "ToyWalker.h"

#include "ikfast.h"

template <size_t JOINTS>
class IkFastSolutionArray : public ikfast::IkSolutionListBase<double>
{
public:
	IkFastSolutionArray()
	: numSolutions(0)
	{ }
	virtual ~IkFastSolutionArray() {}

	virtual size_t AddSolution(const ikfast::IkSingleDOFSolutionBase<double> * vinfos, const int * vfree)
	{
		if (numSolutions < sizeof(solutions) / sizeof(solutions[0])) {
			memcpy(solutions[numSolutions].vinfos, vinfos, sizeof(solutions[numSolutions].vinfos));
			return numSolutions ++;
		} else {
			return numSolutions - 1;
		}
	}
	virtual const ikfast::IkSolutionBase<double>& GetSolution(size_t index) const
	{
		return solutions[index];
	}
	virtual size_t GetNumSolutions() const
	{
		return numSolutions;
	}
	virtual void Clear()
	{
		numSolutions = 0;
	}

	static IkFastSolutionArray<JOINTS> instance;

private:
	size_t numSolutions;
	struct Solution : public ikfast::IkSolutionBase<double>
	{
		void GetSolution(double * solution, const double * freevalues) const
		{
			for (std::size_t i = 0; i < JOINTS; ++ i)
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
			return JOINTS;
		}

		ikfast::IkSingleDOFSolutionBase<double> vinfos[JOINTS];
	} solutions[8];

};

