#include "IkFastSolutionArray.hpp"

#include "Limb.hpp"

namespace toywalker {

template<>
IkFastSolutionArray<Limb::MAX_JOINTS> IkFastSolutionArray<Limb::MAX_JOINTS>::instance{};

}
