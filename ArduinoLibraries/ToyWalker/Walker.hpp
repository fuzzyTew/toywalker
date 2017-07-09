#pragma once

#include <tuple>

#include <Eigen/Geometry>

template <typename... Limbs>
class Walker
{
public:
	std::tuple<Limbs...> limbs;

	void limbGo(unsigned int limb, Eigen::Vector3d const & pos)
	{
		limbGoImpl<Limbs...>(limb, pos);
	}

private:
	template <int basecase = 0>
	inline void limbGoImpl(unsigned int, Eigen::Vector3d const &)
	{ }

	template <typename Limb, typename... Remaining>
	inline void limbGoImpl(unsigned int limb, Eigen::Vector3d const & pos)
	{
		constexpr int i = sizeof...(Limbs) - sizeof...(Remaining) - 1;
		if (i == limb)
			std::get<i>(limbs).go(pos);
		else
			limbGoImpl<Remaining...>(limb, pos);
	}
};
